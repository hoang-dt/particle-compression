// multiresolution-tree.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// TODO: better memory allocation (to put the leaves on the same memory block)
// TODO: memory deallocation?

#define _CRT_SECURE_NO_WARNINGS
#undef min
#undef max
#undef near
#undef far

#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#include "doctest.h"
#include "yocto_math.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <inttypes.h>
#include <random>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>


#if defined(_MSC_VER) // Microsoft compilers
#define EXPAND(X) X
#define __NARGS(_1, _2, _3, _4, _5, VAL, ...) VAL
#define NARGS_1(...) EXPAND(__NARGS(__VA_ARGS__, 4, 3, 2, 1, 0))
#define AUGMENTER(...) unused, __VA_ARGS__
#define NARGS(...) NARGS_1(AUGMENTER(__VA_ARGS__))
#else // Others
#define NARGS(...) __NARGS(0, ## __VA_ARGS__, 5,4,3,2,1,0)
#define __NARGS(_0,_1,_2,_3,_4,_5,N,...) N
#endif

/* Macro overloading feature support */
#define CAT(A, ...) CAT_HELPER(A, __VA_ARGS__)
#define CAT_HELPER(A, ...) A ## __VA_ARGS__
#define OVERLOAD_SELECT(Name, Num) CAT(Name ## _, Num)
#define MACRO_OVERLOAD(Name, ...) OVERLOAD_SELECT(Name, NARGS(__VA_ARGS__))(__VA_ARGS__)
// Examples:
// #define FOO(...)       MACRO_OVERLOAD(FOO, __VA_ARGS__)
// #define FOO_0()        "Zero"
// #define FOO_1(X)       "One"
// #define FOO_2(X, Y)    "Two"
// #define FOO_3(X, Y, Z) "Three"

#define FOR(Type, It, Begin, End) for (Type It = Begin; It != End; ++It)
#define FOR_EACH(...) MACRO_OVERLOAD(FOR_EACH, __VA_ARGS__)
#define FOR_EACH_2(It, Container) for (auto It = begin(Container); It != end(Container); ++It)
#define FOR_EACH_3(It, Begin, End) for (auto It = Begin; It != End; ++It)
#define MAX(A, B) (B) < (A) ? (A) : (B)
#define MIN(A, B) (A) < (B) ? (A) : (B)

#if defined(_MSC_VER)
#define INLINE __forceinline
#elif defined(__clang__) || defined(__GNUC__)
#define INLINE inline __attribute__((__always_inline__))
#endif

#define MCOPY(A, Expr) [&]() { auto __B = A; __B Expr; return __B; }()

using namespace yocto;

using i8  = int8_t;
using u8  = uint8_t;
using i32 = int32_t;
using u32 = uint32_t;
using i64 = int64_t;
using u64 = uint64_t;
using ulong = unsigned long;
using uchar = unsigned char;
using cstr = const char*;

/* Msb and Lsb */
#if defined(__clang__) || defined(__GNUC__)
INLINE i8 Msb(u32 V, i8 Def = -1) { return (V == 0) ? Def : i8(sizeof(u32) * 8 - 1 - __builtin_clz(V)); }
INLINE i8 Msb(u64 V, i8 Def = -1) { return (V == 0) ? Def : i8(sizeof(u64) * 8 - 1 - __builtin_clzll(V)); }
INLINE i8 Lsb(u32 V, i8 Def = -1) { return (V == 0) ? Def : i8(__builtin_ctz(V)); }
INLINE i8 Lsb(u64 V, i8 Def = -1) { return (V == 0) ? Def : i8(__builtin_ctzll(V)); }
#elif defined(_MSC_VER)
#include <intrin.h>
#pragma intrinsic(_BitScanReverse)
#pragma intrinsic(_BitScanReverse64)
INLINE i8 Msb(u32 V, i8 Def = -1) { ulong Idx; uchar Ret =   _BitScanReverse(&Idx, V); return Ret ? (i8)Idx : Def; }
INLINE i8 Msb(u64 V, i8 Def = -1) { ulong Idx; uchar Ret = _BitScanReverse64(&Idx, V); return Ret ? (i8)Idx : Def; }
#pragma intrinsic(_BitScanForward)
#pragma intrinsic(_BitScanForward64)
INLINE i8 Lsb(u32 V, i8 Def = -1) { ulong Idx; uchar Ret =   _BitScanForward(&Idx, V); return Ret ? (i8)Idx : Def; }
INLINE i8 Lsb(u64 V, i8 Def = -1) { ulong Idx; uchar Ret = _BitScanForward64(&Idx, V); return Ret ? (i8)Idx : Def; }
#endif

/* File system stuffs */
#if defined(_WIN32)
#include <direct.h>
#include <io.h>
#include <Windows.h>
#define GetCurrentDir _getcwd
#define MkDir(Dir) _mkdir(Dir)
#define Access(Dir) _access(Dir, 0)
#elif defined(__linux__) || defined(__APPLE__)
#include <sys/stat.h>
#include <unistd.h>
#define GetCurrentDir getcwd
#define MkDir(Dir) mkdir(Dir, 0733)
#define Access(Dir) access(Dir, F_OK)
#endif

inline thread_local char ScratchBuf[1024]; // for temporary strings
#define PRINT(Format, ...) (snprintf(ScratchBuf, sizeof(ScratchBuf), Format, ##__VA_ARGS__), ScratchBuf)

#define mg_RAII(...) mg_MacroOverload(mg_RAII, __VA_ARGS__)
#define mg_RAII_3(Type, Var, Init) Type Var; mg_CleanUp_1(Dealloc(&Var)); Init;
#define mg_RAII_4(Type, Var, Init, Clean) Type Var; mg_CleanUp_1(Clean); Init;

constexpr int NDims = 2;

struct buffer;

struct allocator {
  virtual bool Alloc(buffer* Buf, i64 Bytes) = 0;
  virtual void Dealloc(buffer* Buf) = 0;
  virtual void DeallocAll() = 0;
  virtual ~allocator() {}
};

/* Allocators that know if they own an address/buffer */
struct owning_allocator : allocator {
  virtual bool Own(const buffer& Buf) const = 0;
};

struct mallocator;
/* A simple allocator that allocates simply by bumping a counter */
struct linear_allocator;
/* A linear allocator that uses stack storage. */
template <int Capacity>
struct stack_linear_allocator;
/*
Whenever an allocation of a size in a specific range is made, return the block
immediately from the head of a linked list. Otherwise forward the allocation to
some Parent allocator. */
struct free_list_allocator;
/* Try to allocate using one allocator first (the Primary), then if that fails,
 * use another allocator (the Secondary). */
struct fallback_allocator;

struct mallocator : public allocator {
  bool Alloc(buffer* Buf, i64 Bytes) override;
  void Dealloc(buffer* Buf) override;
  void DeallocAll() override;
};

static inline mallocator& Mallocator() {
  static mallocator Instance;
  return Instance;
}

template <typename t> struct buffer_t;

struct buffer {
  byte* Data = nullptr;
  i64 Bytes = 0;
  allocator* Alloc = nullptr;
  buffer(allocator* AllocIn = &Mallocator());
  template <typename t, int N> buffer(t (&Arr)[N]);
  buffer(const byte* DataIn, i64 BytesIn, allocator* AllocIn = nullptr);
  template <typename t> buffer(const buffer_t<t>& Buf);
  byte& operator[](i64 Idx) const;
  explicit operator bool() const;
  bool operator!=(const buffer& Other) const;
};

struct linear_allocator : public owning_allocator {
  buffer Block;
  i64 CurrentBytes = 0;
  linear_allocator();
  linear_allocator(const buffer& Buf);
  bool Alloc(buffer* Buf, i64 Bytes) override;
  void Dealloc(buffer* Buf) override;
  void DeallocAll() override;
  bool Own(const buffer& Buf) const override;
};

template <int Capacity>
struct stack_linear_allocator : public linear_allocator {
  std::array<byte, Capacity> Storage;
  stack_linear_allocator() : linear_allocator(buffer{Storage.Arr, Capacity}) {}
};

struct free_list_allocator : public allocator {
  struct node { node* Next = nullptr; };
  node* Head = nullptr;
  i64 MinBytes = 0;
  i64 MaxBytes = 0;
  allocator* Parent = nullptr;
  free_list_allocator();
  free_list_allocator(i64 MinBytesIn, i64 MaxBytesIn, allocator* ParentIn = &Mallocator());
  free_list_allocator(i64 Bytes, allocator* ParentIn = &Mallocator());
  bool Alloc(buffer* Buf, i64 Bytes) override;
  void Dealloc(buffer* Buf) override;
  void DeallocAll() override;
};

struct fallback_allocator : public allocator {
  owning_allocator* Primary = nullptr;
  allocator* Secondary = nullptr;
  fallback_allocator();
  fallback_allocator(owning_allocator* PrimaryIn, allocator* SecondaryIn);
  bool Alloc(buffer* Buf, i64 Bytes) override;
  void Dealloc(buffer* Buf) override;
  void DeallocAll() override;
};

void Clone(const buffer& Src, buffer* Dst, allocator* Alloc = &Mallocator());

/* Abstract away memory allocations/deallocations */
template <typename t> void AllocPtr  (t* Ptr, i64 Size, allocator* Alloc = &Mallocator());
template <typename t> void CallocPtr (t* Ptr, i64 Size, allocator* Alloc = &Mallocator());
template <typename t> void DeallocPtr(t* Ptr);

void AllocBuf  (buffer* Buf, i64 Bytes, allocator* Alloc = &Mallocator());
void CallocBuf (buffer* Buf, i64 Bytes, allocator* Alloc = &Mallocator());
void DeallocBuf(buffer* Buf);

template <typename t> void AllocBufT  (buffer_t<t>* Buf, i64 Size, allocator* Alloc = &Mallocator());
template <typename t> void CallocBufT (buffer_t<t>* Buf, i64 Size, allocator* Alloc = &Mallocator());
template <typename t> void DeallocBufT(buffer_t<t>* Buf);

/* Zero out a buffer */
// TODO: replace with the call to Fill, or Memset
void ZeroBuf(buffer* Buf);
template <typename t> void ZeroBufT(buffer_t<t>* Buf);

/* Copy one buffer to another. Here the order of arguments are the reverse of memcpy. */
i64 MemCopy(const buffer& Src, buffer* Dst);
i64 MemCopy(const buffer& Src, buffer* Dst, u64 Bytes);

i64 Size(const buffer& Buf);
void Resize(buffer* Buf, i64 Size, allocator* Alloc = &Mallocator());

bool operator==(const buffer& Buf1, const buffer& Buf2);
buffer operator+(const buffer& Buf, i64 Bytes);

template <typename t>
struct buffer_t {
  t* Data = nullptr;
  i64 Size = 0;
  allocator* Alloc = nullptr;
  buffer_t();
  template <int N>
  buffer_t(t (&Arr)[N]);
  buffer_t(const t* DataIn, i64 SizeIn, allocator* AllocIn = nullptr);
  buffer_t(const buffer& Buf);
  t& operator[](i64 Idx) const;
  explicit operator bool() const;
};

template <typename t> i64 Size(const buffer_t<t>& Buf);
template <typename t> i64 Bytes(const buffer_t<t>& Buf);

template <typename t> void
AllocBufT(buffer_t<t>* Buf, i64 Size, allocator* Alloc) {
  buffer RawBuf;
  AllocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  Buf->Data = (t*)RawBuf.Data;
  Buf->Size = Size;
  Buf->Alloc = Alloc;
}

template <typename t> void
CallocBufT(buffer_t<t>* Buf, i64 Size, allocator* Alloc) {
  buffer RawBuf;
  CallocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  Buf->Data = (t*)RawBuf.Data;
  Buf->Size = Size;
  Buf->Alloc = Alloc;
}

template <typename t> void
DeallocBufT(buffer_t<t>* Buf) {
  buffer RawBuf{(byte*)Buf->Data, i64(Buf->Size * sizeof(t)), Buf->Alloc};
  DeallocBuf(&RawBuf);
  Buf->Data  = nullptr;
  Buf->Size  = 0;
  Buf->Alloc = nullptr;
}

template <typename t> void
AllocPtr(t** Ptr, i64 Size, allocator* Alloc = &Mallocator()) {
  buffer RawBuf;
  AllocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  *Ptr = (t*)RawBuf.Data;
}

template <typename t> void
CallocPtr(t** Ptr, i64 Size, allocator* Alloc = &Mallocator()) {
  buffer RawBuf;
  CallocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  *Ptr = (t*)RawBuf.Data;
}

template <typename t> void
DeallocPtr(t** Ptr, allocator* Alloc = &Mallocator()) {
  buffer RawBuf{(byte*)(*Ptr), 1, Alloc};
  DeallocBuf(&RawBuf);
}

INLINE buffer::
buffer(allocator* AllocIn)
  : Alloc(AllocIn) {}

INLINE buffer::
buffer(const byte* DataIn, i64 BytesIn, allocator* AllocIn)
  : Data(const_cast<byte*>(DataIn)), Bytes(BytesIn), Alloc(AllocIn) {}

template <typename t, int N> INLINE buffer::
buffer(t (&Arr)[N])
  : Data((byte*)const_cast<t*>(&Arr[0])), Bytes(sizeof(Arr)) {}

template <typename t> INLINE buffer::
buffer(const buffer_t<t>& Buf)
  : Data((byte*)const_cast<t*>(Buf.Data))
  , Bytes(Buf.Size * sizeof(t)), Alloc(Buf.Alloc) {}

INLINE byte& buffer::
operator[](i64 Idx) const {
  assert(Idx < Bytes);
  return const_cast<byte&>(Data[Idx]);
}

INLINE bool buffer::
operator!=(const buffer& Other) const {
  return Data != Other.Data || Bytes != Other.Bytes;
}

INLINE buffer::
operator bool() const { return this->Data && this->Bytes; }

INLINE bool
operator==(const buffer& Buf1, const buffer& Buf2) {
  return Buf1.Data == Buf2.Data && Buf1.Bytes == Buf2.Bytes;
}

INLINE i64
Size(const buffer& Buf) { return Buf.Bytes; }

INLINE void
Resize(buffer* Buf, i64 NewSize, allocator* Alloc) {
  if (Size(*Buf) < NewSize) {
    if (Size(*Buf) > 0)
      DeallocBuf(Buf);
    AllocBuf(Buf, NewSize, Alloc);
  }
}

/* typed_buffer stuffs */
template <typename t> INLINE buffer_t<t>::
buffer_t() = default;

template <typename t> template <int N> INLINE buffer_t<t>::
buffer_t(t (&Arr)[N])
  : Data(&Arr[0]), Size(N) {}

template <typename t> INLINE buffer_t<t>::
buffer_t(const t* DataIn, i64 SizeIn, allocator* AllocIn)
  : Data(const_cast<t*>(DataIn)), Size(SizeIn), Alloc(AllocIn) {}

template <typename t> INLINE buffer_t<t>::
buffer_t(const buffer& Buf)
  : Data((t*)const_cast<byte*>(Buf.Data))
  , Size(Buf.Bytes / sizeof(t)), Alloc(Buf.Alloc) {}

template <typename t> INLINE t& buffer_t<t>::
operator[](i64 Idx) const {
  assert(Idx < Size);
  return const_cast<t&>(Data[Idx]);
}

template <typename t> INLINE i64
Size(const buffer_t<t>& Buf) { return Buf.Size; }

template <typename t> INLINE i64
Bytes(const buffer_t<t>& Buf) { return Buf.Size * sizeof(t); }

template <typename t> INLINE buffer_t<t>::
operator bool() const { return Data && Size; }

/* Support only either reading or writing, not both at the same time */
struct bitstream {
  buffer Stream = {};
  byte* BitPtr = nullptr; // Pointer to current byte
  u64 BitBuf = 0; // buffer
  int BitPos = 0; // how many of those bits we've consumed/written

  inline static std::array<u64, 65> Masks = []() {
    std::array<u64, 65> Masks;
    for (int I = 0; I < 64; ++I)
      Masks[I] = (u64(1) << I) - 1;
    Masks[64] = ~u64(0);
    return Masks;
  }();
};

void Rewind    (bitstream* Bs);
i64  Size      (const bitstream& Bs);
i64  BitSize   (const bitstream& Bs);
int  BufferSize(const bitstream& Bs);
/* ---------------- Read functions ---------------- */
void InitRead(bitstream* Bs, const buffer& Stream);
/* Refill our buffer (replace the consumed bytes with new bytes from memory) */
void Refill(bitstream* Bs);
/*
Peek the next "Count" bits from the buffer without consuming them
(Count <= 64 - BitPos). This is often called after Refill(). */
u64 Peek(bitstream* Bs, int Count = 1);
/* Consume the next "Count" bits from the buffer (Count <= 64 - 7).
This is often called after Refill() and potentially Peek(). */
void Consume(bitstream* Bs, int Count = 1);
/*
Extract "Count" bits from the stream (Count <= 64 - 7). This performs at most
one Refill() call. The restriction on Count is due to the fact that Refill()
works in units of bytes, so at most 7 already consumed bits can be left over. */
u64 Read(bitstream* Bs, int Count = 1);
/* Similar to Read() but Count is less restrictive (Count <= 64) */
u64 ReadLong(bitstream* Bs, int Count);

/* ---------------- Write functions ---------------- */
void InitWrite(bitstream* Bs, i64 Bytes, allocator* Alloc = &Mallocator());
void InitWrite(bitstream* Bs, const buffer& Buf);
/* Flush the written BYTES in our buffer to memory */
void Flush(bitstream* Bs);
/* Flush and move the pointer to the next byte in memory */
void FlushAndMoveToNextByte(bitstream* Bs);
/* Put "Count" bits into the buffer (Count <= 64 - BitPos) */
void Put(bitstream* Bs, u64 N, int Count = 1);
/* Write "Count" bits into the stream (Count <= 64 - 7) */
u64  Write(bitstream* Bs, u64 N, int Count = 1);
/* Similar to Write() but Count is less restrictive (Count <= 64) */
u64  WriteLong(bitstream* Bs, u64 N, int Count);
/* Flush and write stream Src to Bs, at byte granularity */
void WriteStream(bitstream* Bs, bitstream* Src);
void WriteBuffer(bitstream* Bs, const buffer& Src);
/* Write "Count" bits into the stream (Count >= 0) */
void RepeatedWrite(bitstream* Bs, bool B, int Count);
/* Zero out the whole buffer */
void Zero(bitstream* Bs);
/* Pad the stream with 0s until a specified number of bits */
void Pad0sUntil(bitstream* Bs, i64 BitCount);

/* Grow the underlying buffer if it is somewhat full */
void GrowIfTooFull(bitstream* Bs);
void GrowToAccomodate(bitstream* Bs, i64 AddedCapacity);
/* Grow the underlying buffer */
void IncreaseCapacity(bitstream* Bs, i64 NewCapacity);

/* Seek to a given byte offset from the start of the stream */
void SeekToByte(bitstream* Bs, i64 ByteOffset);
void SeekToNextByte(bitstream* Bs);
/* Seek to a given bit offset from the start of the stream */
void SeekToBit(bitstream* Bs, i64 BitOffset);
buffer ToBuffer(const bitstream& Bs);

// Only call this if the bit stream itself manages its memory
void Dealloc(bitstream* Bs);

INLINE void
Rewind(bitstream* Bs) {
  Bs->BitPtr = Bs->Stream.Data;
  Bs->BitBuf = Bs->BitPos = 0;
}

INLINE i64
Size(const bitstream& Bs) { return (Bs.BitPtr - Bs.Stream.Data) + (Bs.BitPos + 7) / 8; }

INLINE i64
BitSize(const bitstream& Bs) { return (Bs.BitPtr - Bs.Stream.Data) * 8 + Bs.BitPos; }

INLINE int
BufferSize(const bitstream& Bs) { return sizeof(Bs.BitBuf); }

INLINE void
InitRead(bitstream* Bs, const buffer& Stream) {
  assert(!Stream.Data || Stream.Bytes > 0);
  Bs->Stream = Stream;
  Rewind(Bs);
  Refill(Bs);
}

INLINE void
Refill(bitstream* Bs) {
  assert(Bs->BitPos <= 64);
  Bs->BitPtr += Bs->BitPos >> 3; // ignore the bytes we've consumed
  Bs->BitBuf = *(u64*)Bs->BitPtr; // refill
  Bs->BitPos &= 7; // (% 8) left over bits that don't make a full byte
}

INLINE u64
Peek(bitstream* Bs, int Count) {
  assert(Count >= 0 && Bs->BitPos + Count <= 64);
  u64 Remaining = Bs->BitBuf >> Bs->BitPos; // the bits we have not consumed
  return Remaining & bitstream::Masks[Count]; // return the bottom count bits
}

INLINE void
Consume(bitstream* Bs, int Count) {
  assert(Count + Bs->BitPos <= 64);
  Bs->BitPos += Count;
}

INLINE u64
Read(bitstream* Bs, int Count) {
  assert(Count >= 0 && Count <= 64 - 7);
  if (Count + Bs->BitPos > 64)
    Refill(Bs);
  u64 Result = Peek(Bs, Count);
  Consume(Bs, Count);
  return Result;
}

INLINE u64
ReadLong(bitstream* Bs, int Count) {
  assert(Count >= 0 && Count <= 64);
  int FirstBatchCount = MIN(Count, 64 - Bs->BitPos);
  u64 Result = Peek(Bs, FirstBatchCount);
  Consume(Bs, FirstBatchCount);
  if (Count > FirstBatchCount) {
    Refill(Bs);
    Result |= Peek(Bs, Count - FirstBatchCount) << FirstBatchCount;
    Consume(Bs, Count - FirstBatchCount);
  }
  return Result;
}

INLINE void
InitWrite(bitstream* Bs, const buffer& Buf) {
  assert((size_t)Buf.Bytes >= sizeof(Bs->BitBuf));
  Bs->Stream = Buf;
  Bs->BitPtr = Buf.Data;
  Bs->BitBuf = Bs->BitPos = 0;
}

INLINE void
InitWrite(bitstream* Bs, i64 Bytes, allocator* Alloc) {
  Alloc->Alloc(&Bs->Stream, Bytes + sizeof(Bs->BitBuf));
  Bs->BitPtr = Bs->Stream.Data;
  Bs->BitBuf = Bs->BitPos = 0;
}

INLINE void
Flush(bitstream* Bs) {
  assert(Bs->BitPos <= 64);
  /* write the buffer to memory */
  *(u64*)Bs->BitPtr = Bs->BitBuf; // TODO: make sure this write is in little-endian
  int BytePos = Bs->BitPos >> 3; // number of bytes in the buffer we have used
  /* shift the buffer to the right (the convoluted logic is to avoid shifting by 64 bits) */
  if (BytePos > 0)
    Bs->BitBuf = (Bs->BitBuf >> 1) >> ((BytePos << 3) - 1);
  Bs->BitPtr += BytePos; // advance the pointer
  Bs->BitPos &= 7; // % 8
}

inline void
FlushAndMoveToNextByte(bitstream* Bs) {
  *(u64*)Bs->BitPtr = Bs->BitBuf;
  int BytePos = Bs->BitPos >> 3;
  Bs->BitPtr += BytePos + ((Bs->BitPos & 0x7) != 0); // advance the pointer
  Bs->BitBuf = Bs->BitPos = 0;
}

INLINE void
Put(bitstream* Bs, u64 N, int Count) {
  assert(Count >= 0 && Bs->BitPos + Count <= 64);
  Bs->BitBuf |= (N & bitstream::Masks[Count]) << Bs->BitPos;
  Bs->BitPos += Count;
}

INLINE u64
Write(bitstream* Bs, u64 N, int Count) {
  assert(Count >= 0 && Count <= 64 - 7);
  if (Count + Bs->BitPos >= 64)
    Flush(Bs);
  Put(Bs, N, Count);
  return N;
}

INLINE u64
WriteLong(bitstream* Bs, u64 N, int Count) {
  assert(Count >= 0 && Count <= 64);
  int FirstBatchCount = MIN(Count, 64 - Bs->BitPos);
  Put(Bs, N, FirstBatchCount);
  if (Count > FirstBatchCount) {
    Flush(Bs);
    Put(Bs, N >> FirstBatchCount, Count - FirstBatchCount);
  }
  return N;
}

inline void
WriteStream(bitstream* Bs, bitstream* Src) {
  FlushAndMoveToNextByte(Bs);
  Flush(Src);
  buffer Dst = Bs->Stream + Size(*Bs);
  MemCopy(ToBuffer(*Src), &Dst);
  Bs->BitPtr += Size(*Src);
}

inline void
WriteBuffer(bitstream* Bs, const buffer& Src) {
  FlushAndMoveToNextByte(Bs);
  buffer Dst = Bs->Stream + Size(*Bs);
  MemCopy(Src, &Dst);
  Bs->BitPtr += Size(Src);
}

INLINE void
RepeatedWrite(bitstream* Bs, bool B, int Count) {
  assert(Count >= 0);
  u64 N = ~(u64(B) - 1);
  if (Count <= 64 - 7) { // write at most 57 bits
    Write(Bs, N, Count);
  } else { // write more than 57 bits
    while (true) {
      int NBits = 64 - Bs->BitPos;
      if (NBits <= Count) {
        Put(Bs, N, NBits);
        Count -= NBits;
        Flush(Bs);
      } else {
        Put(Bs, N, Count);
        break;
      }
    }
  }
}

INLINE void
SeekToByte(bitstream* Bs, i64 ByteOffset) {
  Bs->BitPtr = Bs->Stream.Data + ByteOffset;
  Bs->BitBuf = *(u64*)Bs->BitPtr; // refill
  Bs->BitPos = 0;
}

INLINE void
SeekToNextByte(bitstream* Bs) {
  SeekToByte(Bs, Bs->BitPtr - Bs->Stream.Data + ((Bs->BitPos + 7) >> 3));
}

INLINE void
SeekToBit(bitstream* Bs, i64 BitOffset) {
  Bs->BitPtr = Bs->Stream.Data + (BitOffset >> 3);
  Bs->BitBuf = *(u64*)Bs->BitPtr; // refill
  Bs->BitPos = (BitOffset & 7); // (% 8)
}

INLINE buffer
ToBuffer(const bitstream& Bs) {
    return buffer{Bs.Stream.Data, Size(Bs), Bs.Stream.Alloc};
}

INLINE void
Dealloc(bitstream* Bs) {
  Bs->Stream.Alloc->Dealloc(&(Bs->Stream));
  Bs->BitPtr = Bs->Stream.Data;
  Bs->BitBuf = Bs->BitPos = 0;
}

INLINE void
Zero(bitstream* Bs) {
  ZeroBuf(&(Bs->Stream));
  Bs->BitBuf = 0;
}

INLINE void
Pad0sUntil(bitstream* Bs, i64 BitCount) {
  RepeatedWrite(Bs, false, int(BitCount - BitSize(*Bs)));
}

INLINE void
GrowIfTooFull(bitstream* Bs) {
  if (Size(*Bs) * 10 > Size(Bs->Stream) * 8) { // we grow at 80% capacity
    auto NewCapacity = (Size(Bs->Stream) * 3) / 2 + 8;
    IncreaseCapacity(Bs, NewCapacity);
  }
}

INLINE void
GrowToAccomodate(bitstream* Bs, i64 AddedCapacity) {
  i64 OriginalCapacity = Size(Bs->Stream);
  i64 NewCapacity = OriginalCapacity;
  while (Size(*Bs) + AddedCapacity + (i64)sizeof(Bs->BitBuf) >= NewCapacity)
    NewCapacity = (NewCapacity * 3) / 2 + 8;
  if (NewCapacity > OriginalCapacity)
    IncreaseCapacity(Bs, NewCapacity);
}

INLINE void
IncreaseCapacity(bitstream* Bs, i64 NewCapacity) {    
  NewCapacity += sizeof(Bs->BitBuf);
  if (Size(Bs->Stream) < NewCapacity) {
    buffer NewBuf;
    AllocBuf(&NewBuf, NewCapacity, Bs->Stream.Alloc);
    MemCopy(Bs->Stream, &NewBuf, Size(*Bs));
    Bs->BitPtr = (Bs->BitPtr - Bs->Stream.Data) + NewBuf.Data;
    DeallocBuf(&Bs->Stream);
    Bs->Stream = NewBuf;
  }
}

i64
MemCopy(const buffer& Src, buffer* Dst) {
  assert(Dst->Data);
  assert(Src.Data || Src.Bytes == 0);
  assert(Dst->Bytes >= Src.Bytes);
  memcpy(Dst->Data, Src.Data, size_t(Src.Bytes));
  return Src.Bytes;
}

i64
MemCopy(const buffer& Src, buffer* Dst, u64 Bytes) {
  assert(Dst->Data);
  assert(Src.Data || Src.Bytes == 0);
  assert(Dst->Bytes >= Src.Bytes);
  memcpy(Dst->Data, Src.Data, size_t(Bytes));
  return Bytes;
}

buffer
operator+(const buffer& Buf, i64 Bytes) {
  return buffer{ Buf.Data + Bytes, Buf.Bytes - Bytes };
}

void
ZeroBuf(buffer* Buf) {
  assert(Buf->Data);
  memset(Buf->Data, 0, size_t(Buf->Bytes));
}

template <typename t> void
ZeroBufT(buffer_t<t>* Buf) {
  assert(Buf->Data);
  memset(Buf->Data, 0, Buf->Size * sizeof(t));
}

void
AllocBuf(buffer* Buf, i64 Bytes, allocator* Alloc) {
  Alloc->Alloc(Buf, Bytes);
}

void
CallocBuf(buffer* Buf, i64 Bytes, allocator* Alloc) {
  assert(!Buf->Data || Buf->Bytes == 0);
  if (Alloc == &Mallocator()) {
    Buf->Data = (byte*)calloc(size_t(Bytes), 1);
  }
  else {
    AllocBuf(Buf, Bytes, Alloc);
    ZeroBuf(Buf);
  }
  Buf->Bytes = Bytes;
  Buf->Alloc = Alloc;
}

void
DeallocBuf(buffer* Buf) {
  assert(Buf->Alloc);
  Buf->Alloc->Dealloc(Buf);
}

bool
mallocator::Alloc(buffer* Buf, i64 Bytes) {
  assert(!Buf->Data || Buf->Bytes == 0);
  Buf->Data = (byte*)malloc(size_t(Bytes));
  Buf->Bytes = Bytes;
  Buf->Alloc = this;
  return true;
}

void
mallocator::Dealloc(buffer* Buf) {
  free(Buf->Data);
  Buf->Data = nullptr;
  Buf->Bytes = 0;
  Buf->Alloc = nullptr;
}

void
mallocator::DeallocAll() { /* empty */ }

linear_allocator::
linear_allocator() = default;

linear_allocator::
linear_allocator(const buffer & Buf) : Block(Buf) {}

bool linear_allocator::
Alloc(buffer * Buf, i64 Bytes) {
  if (CurrentBytes + Bytes <= Block.Bytes) {
    Buf->Data = Block.Data + CurrentBytes;
    Buf->Bytes = Bytes;
    Buf->Alloc = this;
    CurrentBytes += Bytes;
    return true;
  }
  return false;
}

void linear_allocator::
Dealloc(buffer * Buf) {
  if (Buf->Data + Buf->Bytes == Block.Data + CurrentBytes) {
    Buf->Data = nullptr;
    Buf->Bytes = 0;
    Buf->Alloc = nullptr;
    CurrentBytes -= Buf->Bytes;
  }
}

void linear_allocator::
DeallocAll() {
  CurrentBytes = 0;
}

bool linear_allocator::
Own(const buffer & Buf) const {
  return Block.Data <= Buf.Data && Buf.Data < Block.Data + CurrentBytes;
}

free_list_allocator::
free_list_allocator() = default;

free_list_allocator::
free_list_allocator(i64 MinBytesIn, i64 MaxBytesIn, allocator * ParentIn)
  : MinBytes(MinBytesIn)
  , MaxBytes(MaxBytesIn)
  , Parent(ParentIn) {}

free_list_allocator::
free_list_allocator(i64 Bytes, allocator * ParentIn)
  : free_list_allocator(Bytes, Bytes, ParentIn) {}

bool free_list_allocator::
Alloc(buffer * Buf, i64 Bytes) {
  assert(Parent);
  if (MinBytes <= Bytes && Bytes <= MaxBytes && Head) {
    Buf->Data = (byte*)Head;
    Buf->Bytes = Bytes;
    Buf->Alloc = this;
    Head = Head->Next;
    return true;
  }
  bool Result = Parent->Alloc(Buf, MaxBytes);
  Buf->Bytes = Bytes;
  Buf->Alloc = this;
  return Result;
}

void free_list_allocator::
Dealloc(buffer * Buf) {
  assert(Parent);
  if (MinBytes <= Buf->Bytes && Buf->Bytes <= MaxBytes) {
    Buf->Bytes = 0;
    Buf->Alloc = this;
    node* P = (node*)(Buf->Data);
    P->Next = Head;
    Head = P;
  }
  else {
    Parent->Dealloc(Buf);
  }
}

// NOTE: the client may want to call Parent->DeallocateAll() as well
void free_list_allocator::
DeallocAll() {
  assert(Parent);
  while (Head) {
    node* Next = Head->Next;
    buffer Buf((byte*)Head, MaxBytes, Parent);
    Parent->Dealloc(&Buf);
    Head = Next;
  }
}

fallback_allocator::
fallback_allocator() = default;

fallback_allocator::
fallback_allocator(owning_allocator * PrimaryIn, allocator * SecondaryIn)
  : Primary(PrimaryIn)
  , Secondary(SecondaryIn) {}

bool fallback_allocator::
Alloc(buffer * Buf, i64 Size) {
  bool Success = Primary->Alloc(Buf, Size);
  return Success ? Success : Secondary->Alloc(Buf, Size);
}

void fallback_allocator::
Dealloc(buffer * Buf) {
  if (Primary->Own(*Buf))
    return Primary->Dealloc(Buf);
  Secondary->Dealloc(Buf);
}

void fallback_allocator::
DeallocAll() {
  Primary->DeallocAll();
  Secondary->DeallocAll();
}

void
Clone(const buffer & Src, buffer * Dst, allocator * Alloc) {
  if (Dst->Data && Dst->Bytes != Src.Bytes)
    DeallocBuf(Dst);
  if (!Dst->Data && Dst->Bytes == 0)
    Alloc->Alloc(Dst, Src.Bytes);
  MemCopy(Src, Dst);
}

INLINE int
WriteVarByte(bitstream* Bs, u64 Val) {
  int BytesWritten = 0;
  while (++BytesWritten) {
    Write(Bs, Val & 0x7F, 7);
    Val >>= 7;
    Write(Bs, Val != 0);
    if (Val == 0) break;
  }
  return BytesWritten;
}

INLINE u64
ReadVarByte(bitstream* Bs) {
  u64 Val = 0;
  int Shift = 0;
  while (true) {
    Val = (Read(Bs, 7) << (7 * Shift++)) + Val;
    if (Read(Bs) == 0) break;
  }
  return Val;
}

//template <typename t>
//struct range {
//  t Begin, End;
//  INLINE i64 size() const { return End - Begin; }
//  INLINE t  begin() const { return Begin; }
//  INLINE t    end() const { return End  ; }
//};

#define RANGE(...) MACRO_OVERLOAD(RANGE, __VA_ARGS__)
#define RANGE_0()
#define RANGE_1(Container) (Container).begin(), (Container).end()
#define RANGE_2(Begin, End) Begin, End
#define RANGE_3(Container, Begin, End) (Container).begin() + Begin, (Container).begin() + End

struct empty_struct { };

struct bbox { vec3f Min, Max; };

enum node_type { Root, Inner };

struct particle {
  vec3f Pos; // position
  //u64 Code = 0; // 
};

struct grid {
  vec3i From3, Dims3, Stride3;
};

template <node_type R>
struct tree {
  tree<Inner>* Left   = nullptr;
  tree<Inner>* Right  = nullptr;
  i64 Begin = 0, End = 0;
  using bbox_t = std::conditional_t<R == Root, bbox, empty_struct>;
  [[no_unique_address]] bbox_t BBox = bbox_t();
};

/* Read all particles from a XYZ file */
static std::vector<particle>
ReadXYZ(cstr FileName) {
  FILE* Fp = fopen(FileName, "r");
  std::vector<particle> Particles;

  char Line[256];
  fgets(Line, sizeof(Line), Fp);
  u32 NParticles; sscanf(Line, "%" PRIu32, &NParticles);
  Particles.resize(NParticles);
  fgets(Line, sizeof(Line), Fp); // dummy second line
  FOR (int, I, 0, NParticles) {
    fgets(Line, sizeof(Line), Fp);
    vec3f P3; char C;
    sscanf(Line, "%c %f %f %f", &C, &P3.x, &P3.y, &P3.z);
    Particles[I].Pos = P3;
  }
  return Particles;
}

template <typename t> static void
WriteXYZ(cstr FileName, t Begin, t End) {
  FILE* Fp = fopen(FileName, "w");
  auto NParticles = End - Begin;
  fprintf(Fp, "%zu\n", NParticles);
  fprintf(Fp, "dummy\n");
  FOR_EACH (P3, Begin, End) {
    fprintf(Fp, "C %f %f %f\n", P3->x, P3->y, P3->z);
  }
  fclose(Fp);
}

static bbox
ComputeBoundingBox(const std::vector<particle>& Particles) {
  REQUIRE(!Particles.empty());
  bbox BBox;
  BBox.Min = BBox.Max = Particles[0].Pos;
  FOR_EACH (P3, Particles) {
    BBox.Min = min(BBox.Min, P3->Pos);
    BBox.Max = max(BBox.Max, P3->Pos);
  }
  return BBox;
}

// First 4 bits = Level
// Last 60 bits = block id of particle
#define BLOCK_ID(Level, ParticleId, BlockBits) ((u64(Level) << 60) + ((ParticleId) >> ((Level) + (BlockBits))))
#define LEVEL(BlockId) ((BlockId) >> 60)

std::unordered_map<u64, std::vector<i64>> ParticlesLODs;
struct params {
  int BlockBits = 15; // every 2^15 voxels become one block
};

// TODO: first, compute the bounding box
// TODO: then, partition the particles into 2^x * 2^y * 2^z bricks
// TODO: then, build one tree for each brick
// TODO: then, use the maximum tree depth and re-build a tree for each brick (so that each particle has one coordinate)
// TODO: during the second build, encode the numbers into the correct streams, keeping track of the resolution level and also the spatial level
// TODO: during the second build, adjust the size of the block size accordingly as we traverse down the tree
// TODO: writing one bit at a time at the leaf level can be slow. how to make it fast?
// TODO: during refinement, keep two priority queues: one at the brick level and one for all bricks
// 

// TODO: for each block we store the number of particles (put all the numbers together outside of block data)
// TODO: in the block data we store the binomial coding tree
// TODO: each level can be one file
// TODO: at read time, we read the number of particles for all blocks, then decide which block to refine next using a priority queue
// TODO: when refining a block, we can either read the tree for the block or read the number of particles for the block's children
//       this is decided based on whether the per-pixel error is small enough (if each voxel and its children project to one pixel then we do not need to refine more)

using particles = std::vector<std::unordered_map<u64, std::vector<vec3f>>>; // [level] -> [blocks of particles]
using bitstream_map = std::unordered_map<u64, bitstream>;

static void
Refine(particles* Particles) {

}

static std::vector<particle> Particles;
static params Params;
static bbox BBox;
static vec3i Dims3;
static bitstream_map BitStreams;

INLINE u64 // compute the block code from a given node code
BlockCode(u64 Code) {
  return Code >> Params.BlockBits;
}

INLINE void // N is the number of particles under a node
Encode(u64 Code, i64 N) {
  printf("%llu %lld \n", Code, N);
  return;
  auto Bs = BitStreams[BlockCode(Code)];
  GrowToAccomodate(&Bs, 8); 
  WriteVarByte(&Bs, N);
}

static void
BuildTreeLeaf(tree<Inner>* Node, i64 Begin, u64 Code, const grid& Grid, i8 D, i8 NLevels) {
  Node->Begin = Begin;
  Node->End = Begin + 1;
  // TODO: continue the splitting until we reach the desired accuracy
  //Encode(Code, 1);
}

static grid
SplitGrid(const grid& Grid, int D, bool RSplit, bool Right) {
  REQUIRE((Grid.Dims3[D] & 1) == 0);
  grid Out = Grid;
  if (RSplit) { // resolution split
    Out.From3[D] += Right * Out.Stride3[D];
    Out.Dims3[D] /= 2;
    Out.Stride3[D] *= 2;
  } else { // spatial split
    Out.Dims3[D] /= 2;
    Out.From3[D] += Right * Out.Stride3[D] * Out.Dims3[D];
  }
  return Out;
}

template <node_type R> static void
BuildTreeInner(tree<R>* Node, i64 Begin, i64 End, u64 Code, const grid& Grid, i8 D, i8 NLevels, bool RSplit) {
  REQUIRE((Grid.Dims3[D] & 1) == 0);
  Node->Begin = Begin;
  Node->End = End;
  i64 Mid = Begin;
  float W = (BBox.Max[D] - BBox.Min[D]) / Dims3[D];
  if (RSplit) { // resolution split
    auto RPred = [W, D, &Grid](const particle& P) { 
      int Bin = MIN(Dims3[D] - 1, int((P.Pos[D] - BBox.Min[D]) / W));
      REQUIRE((Bin - Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin - Grid.From3[D]) / Grid.Stride3[D];
      return (Bin & 1) == 0; 
    };
    Mid = partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else { // spatial split
    auto SPred = [W, D, &Grid](const particle& P) { 
      int Bin = MIN(Dims3[D] - 1, int((P.Pos[D] - BBox.Min[D]) / W));
      REQUIRE((Bin - Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin - Grid.From3[D]) / Grid.Stride3[D];
      return Bin * 2 < Grid.Dims3[D];
    };
    Mid = partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }
  Encode(Code * 2, Mid - Begin);
  if (Begin < Mid) {
    Node->Left = new tree<Inner>();
    if (Begin + 1 == Mid) { // leaf
      BuildTreeLeaf(Node->Left, Begin, (Code * 2), SplitGrid(Grid, D, RSplit, false), (D + 1) % NDims, NLevels - 1);
    } else { // recurse on the left
      BuildTreeInner(Node->Left, Begin, Mid, (Code * 2), SplitGrid(Grid, D, RSplit, false), (D + 1) % NDims, NLevels - 1, NLevels > 1);
    }
  }
  if (Mid < End) {
    Node->Right = new tree<Inner>();
    if (Mid + 1 == End) { // leaf
      BuildTreeLeaf(Node->Right, Mid, (Code * 2 + 1), SplitGrid(Grid, D, RSplit, true), (D + 1) % NDims, NLevels - 1);
    } else { // recurse on the right
      BuildTreeInner(Node->Right, Mid, End, (Code * 2 + 1), SplitGrid(Grid, D, RSplit, true), (D + 1) % NDims, NLevels - 1, false);
    }
  }
}

/* Return the tree height and the dimensions of the underlying grid (in terms of power of two) */
static vec3i
ComputeGrid(std::vector<particle>* Particles, const bbox& BBox, i64 Begin, i64 End, i8 D) {
  REQUIRE(Begin < End); // this cannot be a leaf node
  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P) { return P.Pos[D] < Middle; };
  i64 Mid = std::partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left = MCOPY(vec3i(0), [D] = 1), LogDims3Right = MCOPY(vec3i(0), [D] = 1);
  if (Begin + 1 < Mid) {
    LogDims3Left = ComputeGrid(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, (D + 1) % NDims);
    ++LogDims3Left[D];
  }
  if (Mid + 1 < End) {
    LogDims3Right = ComputeGrid(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, (D + 1) % NDims);
    ++LogDims3Right[D];
  }
  return max(LogDims3Left, LogDims3Right);
}

static void
RandomLevels(std::vector<vec3f>* Points) {
  std::random_device Rd;
  std::mt19937 G(Rd());
  shuffle(Points->begin(), Points->end(), G);
  auto Size = Points->size();
  FOR (int, I, 0, 5) {
    WriteXYZ(PRINT("random-particles-%d.xyz", I), RANGE(*Points, Size / 2, Size));
    Size /= 2;
  }
}

static void
Handler(const doctest::AssertData& ad) {
  using namespace doctest;

  // uncomment if asserts will be used in a multi-threaded context
  // std::lock_guard<std::mutex> lock(g_mut);

  // here we can choose what to do:
  // - log the failed assert
  // - throw an exception
  // - call std::abort() or std::terminate()

  std::cout << Color::LightGrey << skipPathFromFilename(ad.m_file) << "(" << ad.m_line << "): ";
  std::cout << Color::Red << failureString(ad.m_at) << ": ";

  // handling only normal (comparison and unary) asserts - exceptions-related asserts have been skipped
  if (ad.m_at & assertType::is_normal) {
    std::cout << Color::Cyan << assertString(ad.m_at) << "( " << ad.m_expr << " ) ";
    std::cout << Color::None << (ad.m_threw ? "THREW exception: " : "is NOT correct!\n");
    if (ad.m_threw)
      std::cout << ad.m_exception;
    else
      std::cout << "  values: " << assertString(ad.m_at) << "( " << ad.m_decomp << " )";
  }
  else {
    std::cout << Color::None << "an assert dealing with exceptions has failed!";
  }

  std::cout << std::endl;
}

int
main(int Argc, cstr* Argv) {
  doctest::Context context(Argc, Argv);
  context.setAsDefaultForAssertsOutOfTestCases();
  context.setAssertHandler(Handler);
  if (Argc < 2) {
    fprintf(stderr, "Usage: .exe particle_file");
    exit(1);
  }
  Particles = ReadXYZ(Argv[1]);
  printf("number of particles = %zu\n", Particles.size());
  BBox = ComputeBoundingBox(Particles);
  auto LogDims3 = ComputeGrid(&Particles, BBox, 0, Particles.size(), 0);
  Dims3 = vec3i(1 << LogDims3.x, 1 << LogDims3.y, 1 << LogDims3.z);
  grid Grid{.From3 = vec3i(0), .Dims3 = Dims3, .Stride3 = vec3i(1)};
  int NLevels = 3; // actually number of resolution splits
  tree<Root> Tree;
  printf("bounding box = (%f %f %f) - (%f %f %f)\n", BBox.Min.x, BBox.Min.y, BBox.Min.z, BBox.Max.x, BBox.Max.y, BBox.Max.z);
  printf("log dims 3 = %d %d %d\n", LogDims3.x, LogDims3.y, LogDims3.z);
  Encode(1, Particles.size());
  BuildTreeInner(&Tree, 0, Particles.size(), 1, Grid, 0, NLevels, NLevels > 0);
  //RandomLevels(P, &Particles);
}

