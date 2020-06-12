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
#include "heap.h"
#define SEXPR_IMPLEMENTATION
#include "sexpr.h"
#include "yocto_math.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <inttypes.h>
#include <queue>
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
#define FOR_EACH_2(It, Container) for (auto It = (Container).begin(); It != (Container).end(); ++It)
#define FOR_EACH_3(It, Begin, End) for (auto It = Begin; It != End; ++It)
#define MAX(A, B) (B) < (A) ? (A) : (B)
#define MIN(A, B) (A) < (B) ? (A) : (B)

#define EXPvec3(V) (V).x, (V).y, (V).z
#define PRIvec3f "%f %f %f"
#define PRIvec3i "%d %d %d"

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
using f32 = float;
using f64 = double;
using ulong = unsigned long;
using uchar = unsigned char;
using cstr = const char*;
using str = char*;

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

#define LOG2_FLOOR(X) Msb(u64(X))

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

#define IS_EVEN(X) (((X) & 1) == 0)
#define IS_INT(X) (int(X) == (X))

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

/*
A "view" into a (usually bigger) null-terminated string. A string_ref itself is
not null-terminated.
There are two preferred ways to construct a string_ref from a char[] array:
  - Use the mg_StringRef macro to make string_ref refer to the entire array
  - Use the string_ref(const char*) constructor to refer up to the first NULL */
struct stref {
  union {
    str Ptr = nullptr;
    cstr ConstPtr ;
  };
  int Size = 0;

  stref();
  stref(cstr PtrIn, int SizeIn);
  stref(cstr PtrIn);
  char& operator[](int Idx) const;
  operator bool() const;
}; // struct string_ref

int Size(const stref& Str);

cstr ToString(const stref& Str);
str  Begin   (stref Str);
str  End     (stref Str);
str  RevBegin(stref Str);
str  RevEnd  (stref Str);
bool operator==(const stref& Lhs, const stref& Rhs);

/* Remove spaces at the start of a string */
stref TrimLeft (const stref& Str);
stref TrimRight(const stref& Str);
stref Trim     (const stref& Str);
/*
Return a substring of a given string. The substring starts at Begin and has
length Size. Return the empty string if no proper substring can be constructed
(e.g. Begin >= Str.Size). */
stref SubString(const stref& Str, int Begin, int Size);
/*
Copy the underlying buffer referred to by Src to the one referred to by Dst.
AddNull should be true whenever dst represents a whole string (as opposed to a
substring). If Src is larger than Dst, we copy as many characters as we can. We
always assume that the null character can be optionally added without
overflowing the memory of Dst. */
void Copy(const stref& Src, stref* Dst, bool AddNull = true);
/* Parse a string_ref and return a number */
bool ToInt   (const stref& Str, int* Result);
bool ToDouble(const stref& Str, f64* Result);

/* Tokenize strings without allocating memory */
struct tokenizer {
  stref Input;
  stref Delims;
  int Pos = 0;

  tokenizer();
  tokenizer(const stref& InputIn, const stref& DelimsIn = " \n\t");
}; // struct tokenizer

void  Init (tokenizer* Tk, const stref& Input, const stref& Delims = " \n\t");
stref Next (tokenizer* Tk);
void  Reset(tokenizer* Tk);

INLINE stref::
stref() = default;

INLINE stref::
stref(cstr PtrIn, int SizeIn)
  : ConstPtr(PtrIn), Size(SizeIn) {}

INLINE stref::
stref(cstr PtrIn)
  : ConstPtr(PtrIn), Size(int(strlen(PtrIn))) {}

INLINE char& stref::
operator[](int Idx) const { assert(Idx < Size); return const_cast<char&>(Ptr[Idx]); }

INLINE stref::
operator bool() const { return Ptr != nullptr; }

INLINE int
Size(const stref& Str) { return Str.Size; }

INLINE str Begin   (stref Str) { return Str.Ptr; }
INLINE str End     (stref Str) { return Str.Ptr + Str.Size; }
INLINE str RevBegin(stref Str) { return Str.Ptr + Str.Size - 1; }
INLINE str RevEnd  (stref Str) { return Str.Ptr - 1; }

INLINE tokenizer::
tokenizer() = default;

INLINE tokenizer::
tokenizer(const stref& InputIn, const stref& DelimsIn)
  : Input(InputIn), Delims(DelimsIn), Pos(0) {}

INLINE void
Init(tokenizer* Tk, const stref& Input, const stref& Delims) {
  Tk->Input = Input;
  Tk->Delims = Delims;
  Tk->Pos = 0;
}

constexpr int power_10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 };

bool
ToInt(const stref& Str, int* Result) {
  stref& StrR = const_cast<stref&>(Str);
  if (!StrR || StrR.Size <= 0)
    return false;

  int Mult = 1, Start = 0;
  if (StrR[0] == '-') {
    Mult = -1;
    Start = 1;
  }
  *Result = 0;
  for (int I = 0; I < Str.Size - Start; ++I) {
    int V = StrR[StrR.Size - I - 1] - '0';
    if (V >= 0 && V < 10)
      *Result += Mult * (V * power_10[I]);
    else
      return false;
  }
  return true;
}

bool
ToDouble(const stref& Str, f64* Result) {
  if (!Str || Str.Size <= 0)
    return false;
  char* EndPtr = nullptr;
  *Result = strtod(Str.ConstPtr, &EndPtr);
  bool Failure = errno == ERANGE || EndPtr == Str.ConstPtr || !EndPtr ||
    !(isspace(*EndPtr) || ispunct(*EndPtr) || (*EndPtr) == 0);
  return !Failure;
}

/* Argument parsing code */
bool
OptVal(int NArgs, cstr* Args, cstr Opt, cstr* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      *Val = Args[I + 1];
      return true;
    }
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, int* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0)
      return ToInt(Args[I + 1], Val);
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, i8* Val) {
  int V = *Val;
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      bool Result = ToInt(Args[I + 1], &V);
      *Val = V;
      return Result;
    }
  }
  *Val = V;
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, u8* Val) {
  int IntVal;
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      bool Success = ToInt(Args[I + 1], &IntVal);
      *Val = IntVal;
      return Success;
    }
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, vec3i* Val) {
  for (int I = 0; I + 3 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      return ToInt(Args[I + 1], &Val->x) &&
        ToInt(Args[I + 2], &Val->y) &&
        ToInt(Args[I + 3], &Val->z);
    }
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, std::vector<int>* Vals) {
  Vals->clear();
  for (int I = 0; I < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      int J = I;
      while (true) {
        ++J;
        int X;
        if (J < NArgs&& ToInt(Args[J], &X)) { Vals->push_back(X); }
        else { break; }
      }
      return J > I + 1;
    }
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, vec2i* Val) {
  for (int I = 0; I + 2 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      return ToInt(Args[I + 1], &Val->x) &&
        ToInt(Args[I + 2], &Val->y);
    }
  }
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, f32* Val) {
  f64 DVal = 0;
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      bool Result = ToDouble(Args[I + 1], &DVal);
      *Val = (f32)DVal;
      return Result;
    }
  }
  *Val = (f32)DVal;
  return false;
}

bool
OptVal(int NArgs, cstr* Args, cstr Opt, f64* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0)
      return ToDouble(Args[I + 1], Val);
  }
  return false;
}

bool
OptExists(int NArgs, cstr* Args, cstr Opt) {
  for (int I = 0; I < NArgs; ++I) {
    if (strcmp(Args[I], Opt) == 0)
      return true;
  }
  return false;
}

template <typename func_t>
struct scope_guard {
  func_t Func;
  bool Dismissed = false;
  // TODO: std::forward FuncIn?
  scope_guard(const func_t& FuncIn) : Func(FuncIn) {}
  ~scope_guard() { if (!Dismissed) { Func(); } }
};

#define BEGIN_CLEANUP(...) MACRO_OVERLOAD(BEGIN_CLEANUP, __VA_ARGS__)
#define BEGIN_CLEANUP_0() auto CAT(__CleanUpFunc__, __LINE__) = [&]()
#define BEGIN_CLEANUP_1(N) auto __CleanUpFunc__##N = [&]()
#define END_CLEANUP(...) MACRO_OVERLOAD(END_CLEANUP, __VA_ARGS__)
#define END_CLEANUP_0() scope_guard<decltype(CAT(__CleanUpFunc__, __LINE__))> CAT(__ScopeGuard__, __LINE__)(CAT(__CleanUpFunc__, __LINE__));
#define END_CLEANUP_1(N) scope_guard __ScopeGuard__##N(__CleanUpFunc__##N);

//#define mg_BeginCleanUp(n) auto __CleanUpFunc__##n = [&]()
#define CLEANUP(...) MACRO_OVERLOAD(CLEANUP, __VA_ARGS__)
#define CLEANUP_1(...) BEGIN_CLEANUP_0() { __VA_ARGS__; }; END_CLEANUP_0()
#define CLEANUP_2(N, ...) BEGIN_CLEANUP_1(N) { __VA_ARGS__; }; END_CLEANUP_1(N)
#define DISMISS_CLEANUP(N) { __ScopeGuard__##N.Dismissed = true; }

/* Enable support for reading large files */
#if defined(_WIN32)
  #define FSEEK _fseeki64
  #define FTELL _ftelli64
#elif defined(__linux__) || defined(__APPLE__)
  #define _FILE_OFFSET_BITS 64
  #define FSEEK fseeko
  #define FTELL ftello
#endif


bool
ReadFile(cstr FileName, buffer* Buf) {
  assert((Buf->Data && Buf->Bytes) || (!Buf->Data && !Buf->Bytes));

  FILE* Fp = fopen(FileName, "rb");
  CLEANUP(0, if (Fp) fclose(Fp); );
  if (!Fp) return false;

  /* Determine the file size */
  if (FSEEK(Fp, 0, SEEK_END)) return false;
  i64 Size = 0;
  if ((Size = FTELL(Fp)) == -1) return false;
  if (FSEEK(Fp, 0, SEEK_SET)) return false;
  if (Buf->Bytes < Size)
    AllocBuf(Buf, Size);

  /* Read file contents */
  CLEANUP(1, DeallocBuf(Buf); );
  if (fread(Buf->Data, size_t(Size), 1, Fp) != 1) return false;

  DISMISS_CLEANUP(1);
  return true;
}

template <typename count_t>
struct prob {
  count_t Low; // from 0 to count
  count_t High; // from 0 to count
  count_t Count;
};

/* Arithmetic coder */
/* Condition: CodeValBits >= CountBits + 2 TODO : enforce this condition */
template <typename code_t = u64, typename count_t = u32, int CodeBits = 33, int CountBits = 31>
struct arithmetic_coder {
  static const code_t CodeMax = (code_t(1) << CodeBits) - 1;
  static const code_t CodeOneFourth = code_t(1) << (CodeBits - 2);
  static const code_t CodeOneHalf = 2 * CodeOneFourth;
  static const code_t CodeThreeFourths = 3 * CodeOneFourth;

  code_t CodeLow, CodeHigh, CodeVal;
  int PendingBits;
  bitstream BitStream;

  /*Init for encoding, bytes = the size of the compressed stream in bytes */
  void
  InitWrite(int Bytes) {
    CodeLow = CodeVal = PendingBits = 0;
    CodeHigh = CodeMax;
    InitWrite(&BitStream, Bytes);
  }

  /* Init for decoding */
  void
  InitRead() {
    CodeLow = CodeVal = PendingBits = 0;
    CodeHigh = CodeMax;
    InitRead(&BitStream, BitStream.Stream);
    for (int I = 0; I < CodeBits; ++I) { // TODO: what if we read past the stream?
      CodeVal <<= 1;
      CodeVal += Read(&BitStream);
    }
  }

  /* Make sure low <= val <= high at the end of the stream */
  void
  EncodeFinalize() {
    ++PendingBits;
    if (CodeLow < CodeOneFourth)
      PutBitsPlusPending(0);
    else
      PutBitsPlusPending(1);
    Flush(&BitStream);
  }

  /* Encode a single symbol */
  void
  Encode(const prob<count_t>& P) {
    assert(P.Count > 0);
    code_t Range = CodeHigh - CodeLow + 1;
    CodeHigh = CodeLow + (Range * P.High / P.Count) - 1; // the -1 makes sure new m_code_high <= old m_code_high (== happens when p.high==p.count)
    CodeLow = CodeLow + (Range * P.Low / P.Count);
    /* renormalization */
    while (true) {
      if (CodeHigh < CodeOneHalf) {
        PutBitsPlusPending(0);
      } else if (CodeLow >= CodeOneHalf) {
        PutBitsPlusPending(1);
      } else if (CodeLow >= CodeOneFourth && CodeHigh < CodeThreeFourths) {
        ++PendingBits;
        CodeLow -= CodeOneFourth;
        CodeHigh -= CodeOneFourth;
      } else {
        break;
      }
      CodeHigh <<= 1;
      ++CodeHigh; // shift in a 1-bit on the right
      CodeHigh &= CodeMax; // remove the already shifted bits on the left
      CodeLow <<= 1;
      CodeLow &= CodeMax;
    }
  }

  void
  PutBitsPlusPending(bool Bit) {
    Write(&BitStream, Bit); // TODO: optimize?
    RepeatedWrite(&BitStream, !Bit, PendingBits);
    PendingBits = 0;
  }

  /* Decode a single symbol and return its index in the CDF table */
  size_t
  Decode(const std::vector<count_t>& CdfTable) {
    assert(CdfTable.size() > 0);
    count_t Count = CdfTable[CdfTable.size() - 1];
    assert(Count > 0);
    code_t Range = CodeHigh - CodeLow + 1;
    code_t V = ((CodeVal - CodeLow + 1) * Count - 1) / Range;
    size_t S = 0;
    for (; S < CdfTable.size() && CdfTable[S] <= V; ++S) {}
    count_t Low = S == 0 ? 0 : CdfTable[S - 1];
    count_t High = CdfTable[S];
    CodeHigh = CodeLow + (Range * High) / Count - 1;
    CodeLow = CodeLow + (Range * Low) / Count;

    /* renormalization */
    while (true) {
      if (CodeHigh < CodeOneHalf) {
        // do nothing
      } else if (CodeLow >= CodeOneHalf) {
        CodeVal -= CodeOneHalf;
        CodeLow -= CodeOneHalf;
        CodeHigh -= CodeOneHalf;
      } else if (CodeLow >= CodeOneFourth && CodeHigh < CodeThreeFourths) {
        CodeVal -= CodeOneFourth;
        CodeLow -= CodeOneFourth;
        CodeHigh -= CodeOneFourth;
      } else
        break;
      CodeLow <<= 1;
      CodeHigh <<= 1;
      ++CodeHigh;
      CodeVal <<= 1;
      CodeVal += Read(&BitStream);
    }
    return S;
  }
};

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
  vec3f From3, Dims3, Stride3;
};

struct tree {
  tree* Left   = nullptr;
  tree* Right  = nullptr;
  i64 Begin = 0, End = 0;
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
enum class action : int { Encode, Decode };
struct params {
  char Name[64];
  vec2i Version = vec2i(1, 0);
  int NDims = 3;
  cstr InFile;
  cstr OutFile;
  int BlockBits = 15; // every 2^15 voxels become one block
  i8 NLevels = 3;
  u8 Height = 255; // height of the full tree
  action Action = action::Encode;
  i64 NParticles;
  float Accuracy = 0;
  bbox BBox;
  vec3i LogDims3;
  vec3i Dims3;
};

// TODO: at read time, we read the number of particles for all blocks, then decide which block to refine next using a priority queue
// TODO: when refining a block, we can either read the tree for the block or read the number of particles for the block's children
//       this is decided based on whether the per-pixel error is small enough (if each voxel and its children project to one pixel then we do not need to refine more)

static std::vector<particle> Particles;
static params Params;
static std::vector<bitstream> BlockStreams; // [level] -> bitstream (of the current block)
static std::vector<u64> CurrBlocks; // [level] -> current block id
static std::vector<std::vector<i64>> BlockBytes; // [level] -> [block id] -> block size
static std::vector<std::vector<i64>> BlockOffsets; // [level] -> [block id] -> block offset
struct block {
  std::vector<i64> Nodes;
};
using block_table = std::vector<std::vector<block>>; // [level] -> [block id] -> block data
static block_table Blocks;

#define NODE_TO_BLOCK_INDEX(Idx) (Idx) >> (Params.BlockBits)
#define NODE_INDEX_IN_BLOCK(Idx) (Idx) & ((1ull << Params.BlockBits) - 1)

static bool
ReadMetaFile(cstr FileName) {
  buffer Buf;
  ReadFile(FileName, &Buf);
  CLEANUP(0, DeallocBuf(&Buf));
  SExprResult Result = ParseSExpr((cstr)Buf.Data, Size(Buf), nullptr);
  if(Result.type == SE_SYNTAX_ERROR) {
    fprintf(stderr, "Error(%d): %s.\n", Result.syntaxError.lineNumber, Result.syntaxError.message);
    return false;
  } else { // no syntax error
    SExpr* Data = (SExpr*)malloc(sizeof(SExpr) * Result.count);
    CLEANUP(1, free(Data));
    std::vector<SExpr*> Stack; Stack.reserve(Result.count);
    // This time we supply the pool
    SExprPool Pool = { Result.count, Data };
    Result = ParseSExpr((cstr)Buf.Data, Size(Buf), &Pool);
    // result.expr contains the successfully parsed SExpr
//    printf("parse .idx file successfully\n");
    Stack.push_back(Result.expr);
    bool GotId = false;
    SExpr* LastExpr = nullptr;
    while (Stack.size() > 0) {
      SExpr* Expr = Stack.back();
      Stack.pop_back();
      if (Expr->next)
        Stack.push_back(Expr->next);
      if (GotId) {
        if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "version")) {
          REQUIRE(Expr->type == SE_INT);
          Params.Version[0] = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          REQUIRE(Expr->type == SE_INT);
          Params.Version[1] = Expr->i;
          printf("Version = %d.%d\n", Params.Version[0], Params.Version[1]);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "name")) {
          REQUIRE(Expr->type == SE_STRING);
          snprintf((str)Params.Name, Expr->s.len + 1, "%s", (cstr)Buf.Data + Expr->s.start);
          printf("Name = %s\n", Params.Name);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "grid")) {
          REQUIRE(Expr->type == SE_INT);
          Params.Dims3.x = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          REQUIRE(Expr->type == SE_INT);
          Params.Dims3.y = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          REQUIRE(Expr->type == SE_INT);
          Params.Dims3.z = Expr->i;
          printf("Dims = %d %d %d\n", EXPvec3(Params.Dims3));
        } if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "accuracy")) {
          REQUIRE(Expr->type == SE_FLOAT);
          Params.Accuracy = Expr->f;
          printf("Accuracy = %.8g\n", Params.Accuracy);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "bounding-box")) {
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Min.x = Expr->f;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Min.y = Expr->f;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Min.z = Expr->f;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Max.x = Expr->f;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Max.y = Expr->f;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBox.Max.z = Expr->f;
          printf("bounding-box %f %f %f %f %f %f\n", EXPvec3(Params.BBox.Min), EXPvec3(Params.BBox.Max));
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "particles")) {
          REQUIRE(Expr->type == SE_INT);
          Params.NParticles = Expr->i;
          printf("particles = %lld\n", Params.NParticles);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "resolutions")) {
          REQUIRE(Expr->type == SE_INT);
          Params.NLevels = Expr->i;
          printf("resolutions = %d\n", Params.NLevels);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "block-bits")) {
          REQUIRE(Expr->type == SE_INT);
          Params.BlockBits = Expr->i;
          printf("block-bits = %d\n", Params.BlockBits);
        }
      }
      if (Expr->type == SE_ID) {
        LastExpr = Expr;
        GotId = true;
      } else if (Expr->type == SE_LIST) {
        Stack.push_back(Expr->head);
        GotId = false;
      } else {
        GotId = false;
      }
    }
  }
  return true;
}

static void
WriteMetaFile(const params& Params, cstr FileName) {
  FILE* Fp = fopen(FileName, "w");
  fprintf(Fp, "(\n"); // begin (
  fprintf(Fp, "  (common\n");
  fprintf(Fp, "    (name \"%s\")\n", Params.Name);
  fprintf(Fp, "    (particles %lld)\n", Params.NParticles);
  fprintf(Fp, "    (dimensions %d)\n", Params.NDims);
  fprintf(Fp, "    (grid %d %d %d)\n", EXPvec3(Params.Dims3));
  fprintf(Fp, "    (bounding-box %.10f %.10f %.10f %.10f %.10f %.10f)\n", EXPvec3(Params.BBox.Min), EXPvec3(Params.BBox.Max));
  fprintf(Fp, "  )\n"); // end common)
  fprintf(Fp, "  (format\n");
  fprintf(Fp, "    (version %d %d)\n", Params.Version[0], Params.Version[1]);
  fprintf(Fp, "    (resolutions %d)\n", Params.NLevels);
  fprintf(Fp, "    (block-bits %d)\n", Params.BlockBits);
  fprintf(Fp, "    (accuracy %.10f)\n", Params.Accuracy);
  fprintf(Fp, "    (height %d)\n", Params.Height);
  fprintf(Fp, "  )\n"); // end format)
  fprintf(Fp, ")\n"); // end )
  fclose(Fp);
}

template <typename t> INLINE void WritePOD(FILE* Fp, const t Var) { fwrite(&Var, sizeof(Var), 1, Fp); }
template <typename t> INLINE void WriteBuffer(FILE* Fp, const buffer& Buf) { fwrite(Buf.Data, Size(Buf), 1, Fp); }
template <typename t> INLINE void WriteBuffer(FILE* Fp, const buffer& Buf, i64 Sz) { fwrite(Buf.Data, Sz, 1, Fp); }
template <typename t> INLINE void ReadBuffer(FILE* Fp, buffer* Buf) { fread(Buf->Data, Size(*Buf), 1, Fp); }
template <typename t> INLINE void ReadBuffer(FILE* Fp, buffer* Buf, i64 Sz) { fread(Buf->Data, Sz, 1, Fp); }
template <typename t> INLINE void ReadBuffer(FILE* Fp, buffer_t<t>* Buf) { fread(Buf->Data, Bytes(*Buf), 1, Fp); }
template <typename t> INLINE void ReadPOD(FILE* Fp, t* Val) { fread(Val, sizeof(t), 1, Fp); }
template <typename t> INLINE void
ReadBackwardPOD(FILE* Fp, t* Val) {
  auto Where = FTELL(Fp);
  FSEEK(Fp, Where -= sizeof(t), SEEK_SET);
  fread(Val, sizeof(t), 1, Fp);
  FSEEK(Fp, Where, SEEK_SET);
}
INLINE void
ReadBackwardBuffer(FILE* Fp, buffer* Buf) {
  auto Where = FTELL(Fp);
  FSEEK(Fp, Where -= Size(*Buf), SEEK_SET);
  fread(Buf->Data, Size(*Buf), 1, Fp);
  FSEEK(Fp, Where, SEEK_SET);
}
INLINE void
ReadBackwardBuffer(FILE* Fp, buffer* Buf, i64 Sz) {
  assert(Sz <= Size(*Buf));
  auto Where = FTELL(Fp);
  FSEEK(Fp, Where -= Sz, SEEK_SET);
  fread(Buf->Data, Sz, 1, Fp);
  FSEEK(Fp, Where, SEEK_SET);
}

static bool
ReadResBlock() {
  FILE* Fp = fopen(PRINT("%s-%d.bin", Params.Name, Params.NLevels), "rb");
  FSEEK(Fp, 0, SEEK_END);
  auto Size = FTELL(Fp);
  FSEEK(Fp, 0, SEEK_SET);
  GrowToAccomodate(&BlockStreams[Params.NLevels], Size);
  fread(BlockStreams[Params.NLevels].Stream.Data, Size, 1, Fp);
  fclose(Fp);
  return true;
}

static bool
ReadBlock(i8 Level, u64 BlockId) {
  REQUIRE(Level < Params.NLevels);
  if ((i8)BlockOffsets.size() <= Level) {
    BlockOffsets.resize(Params.NLevels);
    BlockStreams.resize(Params.NLevels);
  }

  FILE* Fp = nullptr;
  if (BlockOffsets[Level].empty()) {
    // read the block bytes
    Fp = fopen(PRINT("%s-%d.bin", Params.Name, Level), "rb");
    if (!Fp)
      return false;
    FSEEK(Fp, 0, SEEK_END);
    u64 NBlocks = 0;
    ReadBackwardPOD(Fp, &NBlocks);
    REQUIRE(BlockId < NBlocks);
    BlockOffsets[Level].resize(NBlocks);
    buffer Buf((byte*)&BlockOffsets[Level][0], (i64)sizeof(BlockOffsets[Level][0]) * NBlocks);
    ReadBackwardBuffer(Fp, &Buf);
    FOR (i64, I, 1, NBlocks) {
      BlockOffsets[Level][I] += BlockOffsets[Level][I - 1];
    }
  }
  if (BlockId >= BlockOffsets[Level].size())
    return false;

  if (!Fp) 
    Fp = fopen(PRINT("%s-%d.bin", Params.Name, Level), "rb");
  FSEEK(Fp, BlockId == 0 ? 0 : BlockOffsets[Level][BlockId - 1], SEEK_SET);
  i64 BlockByte = BlockId == 0 ? BlockOffsets[Level][0] : BlockOffsets[Level][BlockId] - BlockOffsets[Level][BlockId - 1];
  GrowToAccomodate(&BlockStreams[Level], BlockByte);
  if (BlockByte != 0)
    fread(BlockStreams[Level].Stream.Data, BlockByte, 1, Fp);
  else
    return false;
  fclose(Fp);

  return true;
}

INLINE static i64
DecodeNode(bitstream* Bs, i64 M) {
  // TODO: use binomial coding
  return ReadVarByte(Bs);
}

#define RES_PARENT(NodeIdx) ((NodeIdx) - (2 - ((NodeIdx) & 1)))

static void
DecodeResBlock(bitstream* Bs, block* Block) {
  int NNodes = Params.NLevels * 2 - 1;
  Block->Nodes.resize(NNodes);
  InitRead(Bs, Bs->Stream);
  // TODO: use binomial coding
  Block->Nodes[0] = ReadVarByte(Bs);
  for (int I = 2; I < NNodes; I += 2) {
    i64 M = Block->Nodes[RES_PARENT(I)];
    Block->Nodes[I    ] = DecodeNode(Bs, M);
    Block->Nodes[I - 1] = M - Block->Nodes[I];
    printf("%lld %lld\n", Block->Nodes[I-1], Block->Nodes[I]);
    assert(RES_PARENT(I) == RES_PARENT(I - 1));
  }
}

#define RES_LVL_TO_NODE(Level) (((Level) > 0) + (Params.NLevels - 1 - (Level)) * 2)

// TODO: be careful not to confuse blocks of two nodes (left and right children) vs blocks of one nodes (only left child)
static void
DecodeBlock(bitstream* Bs, i8 Level, u64 BlockIdx, block_table* AllBlocks) {
  const block& ResBlock = (*AllBlocks)[Params.NLevels][0];
  InitRead(Bs, Bs->Stream);
  REQUIRE(AllBlocks->size() > Level);
  auto& Blocks = (*AllBlocks)[Level];
  if (Blocks.size() <= BlockIdx) {
    Blocks.resize(BlockIdx * 3 / 2 + 1);
  }
  block& Block = (*AllBlocks)[Level][BlockIdx];
  i64 NNodes = 1ll << Params.BlockBits;
  Block.Nodes.resize(NNodes, 0);
  u64 FirstNodeIdx = MAX(BlockIdx << Params.BlockBits, 2); // NOTE: node index always starts at 2
  u64 LastNodeIdx = (BlockIdx + 1) << Params.BlockBits;
  if (BlockIdx == 0) { // the parent block is the res block
    REQUIRE(Level < Params.NLevels);
    Block.Nodes[1] = ResBlock.Nodes[RES_LVL_TO_NODE(Level)];
  }
  for (u64 K = FirstNodeIdx; K < LastNodeIdx; K += 2) {
    u64 I = NODE_INDEX_IN_BLOCK(K);
    u64 J = K / 2; // (global) parent index
    i64 M = Blocks[NODE_TO_BLOCK_INDEX(J)].Nodes[NODE_INDEX_IN_BLOCK(J)];
    if (M > 0) {
      Block.Nodes[I    ] = DecodeNode(Bs, M); // left child
      Block.Nodes[I + 1] = M - Block.Nodes[I]; // right child
      printf("%lld %lld\n", Block.Nodes[I], Block.Nodes[I+1]);
    }
  }
}

struct block_data {
  i8 Level = 0;
  u64 BlockId = 0;
};
INLINE bool operator<(const block_data& Lhs,  const block_data& Rhs) {
  bool LvlLess   = Lhs.Level > Rhs.Level;
  bool LvlEq     = Lhs.Level == Rhs.Level;
  bool BlockLess = Lhs.BlockId > Rhs.BlockId;
  return LvlLess || (LvlEq && BlockLess);
}

struct block_priority {
  i8 Level = 0;
  f32 Error = 0;
  u64 BlockId = 0;
};
INLINE bool operator<(const block_priority& Lhs, const block_priority& Rhs) {
  bool LvlLess   = Lhs.Level > Rhs.Level;
  bool LvlEq     = Lhs.Level == Rhs.Level;
  bool ErrorLess = Lhs.Error > Rhs.Error;
  bool ErrorEq   = Lhs.Error == Rhs.Error;
  bool BlockLess = Lhs.BlockId > Rhs.BlockId;
  return LvlLess || (LvlEq && (ErrorLess || (ErrorEq && BlockLess)));
}

DynamicHeap<block_data, block_priority> Heap;

// TODO: and from tree depth to bounding box
#define LEVEL_TO_HEIGHT(Level) ((Params.NLevels - (Level)) - ((Level) == 0))
//#define NODE_TO_HEIGHT(Level, BlockIdx, NodeIdx) (LOG2_FLOOR((BlockIdx) * (1ll << Params.BlockBits) + (NodeIdx)) + LEVEL_TO_HEIGHT(Level))
#define NODE_TO_HEIGHT(Level, NodeIdx) (LOG2_FLOOR(NodeIdx) + LEVEL_TO_HEIGHT(Level))

INLINE double
NodeVolume(i8 Level, i64 NodeIdx) {
  vec3f V3 = Params.BBox.Max - Params.BBox.Min;
  float V = V3.x * V3.y * V3.z;
  int H = NODE_TO_HEIGHT(Level, NodeIdx);
  double S = ldexp(V, -H);
  return S;
}

/* Read the next most important block and add its two children (if existed) to the heap
 * Return false if there is no more block to load */
static bool
Refine() {
  block_data TopBlock;
  bool BlockExists = false;
  while (!BlockExists) {
    if (Heap.empty()) break;
    Heap.top(TopBlock);
    if (TopBlock.Level == Params.NLevels)
      BlockExists = ReadResBlock();
    else
      BlockExists = ReadBlock(TopBlock.Level, TopBlock.BlockId);
  }
  if (!BlockExists)
    return false;

  if (TopBlock.Level == Params.NLevels)
    DecodeResBlock(&BlockStreams[TopBlock.Level], &Blocks[TopBlock.Level][0]);
  else
    DecodeBlock(&BlockStreams[TopBlock.Level], TopBlock.Level, TopBlock.BlockId, &Blocks);
//  REQUIRE(LvlBlocks[TopBlock.Level].size() > TopBlock.BlockId);
  block_data LeftChild, RightChild;
  float LeftError = 0, RightError = 0;
  auto& Nodes = Blocks[TopBlock.Level][TopBlock.BlockId].Nodes;
  u64 GlobalFirstNodeIdx = TopBlock.BlockId * (1ll << Params.BlockBits);
  float NodeVol = NodeVolume(TopBlock.Level, GlobalFirstNodeIdx);
  if (TopBlock.Level == Params.NLevels) { // resolution block
    int NNodes = Params.NLevels * 2 - 1;
    REQUIRE(NNodes == Nodes.size());
    FOR (int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      if (NodeIdx + 1 != NNodes && IS_EVEN(NodeIdx))
        continue;
      int Level = (2 * (Params.NLevels - 1) - (NodeIdx - 1)) / 2;
      // NOTE: we have only one child instead of two (BlockBits >= 1)
      LeftChild.Level = Level;
      LeftChild.BlockId = 0;
      LeftError = NodeVol / Nodes[NodeIdx];
      Heap.insert(LeftChild, block_priority{ .Level = LeftChild.Level, .Error = LeftError, .BlockId = LeftChild.BlockId });
    }
  } else { // just a regular block on some resolution
    LeftChild.Level = RightChild.Level = TopBlock.Level;
    LeftChild.BlockId = TopBlock.BlockId * 2 + 1;
    RightChild.BlockId = TopBlock.BlockId * 2 + 2;
    FOR (int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      u64 GlobalNodeIdx = GlobalFirstNodeIdx + NodeIdx;
      u64 ChildrenBlockIdx = NODE_TO_BLOCK_INDEX(GlobalNodeIdx * 2);
      if (ChildrenBlockIdx == TopBlock.BlockId) continue;
      assert(ChildrenBlockIdx == LeftChild.BlockId || ChildrenBlockIdx == RightChild.BlockId);
      if (ChildrenBlockIdx == LeftChild.BlockId)
        LeftError += NodeVol / Nodes[NodeIdx];
      else
        RightError += NodeVol / Nodes[NodeIdx];
    }
    if (LeftError > 0)
      Heap.insert(LeftChild, block_priority{ .Level = LeftChild.Level, .Error = LeftError, .BlockId = LeftChild.BlockId });
    if (RightError > 0)
      Heap.insert(RightChild, block_priority{ .Level = RightChild.Level, .Error = RightError, .BlockId = RightChild.BlockId });
  }

  return true;
}

static void
RefineLeftToRight() {
  // TODO: just refine the tree from left to right, no priority
}

static bool
RefineByLevel() {
  block_data TopBlock;
  bool BlockExists = false;
  while (!BlockExists) {
    if (Heap.empty()) break;
    Heap.top(TopBlock);
    Heap.pop();
    if (TopBlock.Level == Params.NLevels)
      BlockExists = ReadResBlock();
    else
      BlockExists = ReadBlock(TopBlock.Level, TopBlock.BlockId);
  }
  if (!BlockExists)
    return false;

  printf("level %d block %llu\n", TopBlock.Level, TopBlock.BlockId);
  if (TopBlock.Level == Params.NLevels)
    DecodeResBlock(&BlockStreams[TopBlock.Level], &Blocks[TopBlock.Level][0]);
  else
    DecodeBlock(&BlockStreams[TopBlock.Level], TopBlock.Level, TopBlock.BlockId, &Blocks);
//  REQUIRE(LvlBlocks[TopBlock.Level].size() > TopBlock.BlockId);
  block_data LeftChild, RightChild;
  float LeftError = 0, RightError = 0;
  auto& Nodes = Blocks[TopBlock.Level][TopBlock.BlockId].Nodes;
  if (TopBlock.Level == Params.NLevels) { // resolution block
    int NNodes = Params.NLevels * 2 - 1;
    REQUIRE(NNodes == Nodes.size());
    FOR (int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      if (NodeIdx + 1 != NNodes && IS_EVEN(NodeIdx))
        continue;
      int Level = (2 * (Params.NLevels - 1) - (NodeIdx - 1)) / 2;
      // NOTE: we have only one child instead of two (BlockBits >= 1)
      LeftChild.Level = Level;
      LeftChild.BlockId = 0;
      LeftError = 1;
      Heap.insert(LeftChild, block_priority{ .Level = LeftChild.Level, .Error = LeftError, .BlockId = LeftChild.BlockId });
    }
  } else { // just a regular block on some resolution
    LeftChild.Level = RightChild.Level = TopBlock.Level;
    LeftChild.BlockId = TopBlock.BlockId * 2;
    RightChild.BlockId = TopBlock.BlockId * 2 + 1;
    FOR (int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      u64 GlobalNodeIdx = TopBlock.BlockId * (1ll << Params.BlockBits) + NodeIdx;
      u64 ChildrenBlockIdx = NODE_TO_BLOCK_INDEX(GlobalNodeIdx * 2);
      if (ChildrenBlockIdx == TopBlock.BlockId) continue;
      assert(ChildrenBlockIdx == LeftChild.BlockId || ChildrenBlockIdx == RightChild.BlockId);
      if (ChildrenBlockIdx == LeftChild.BlockId)
        LeftError = 1;
      else
        RightError = 1;
    }
    if (LeftError > 0)
      Heap.insert(LeftChild, block_priority{ .Level = LeftChild.Level, .Error = LeftError, .BlockId = LeftChild.BlockId });
    if (RightError > 0)
      Heap.insert(RightChild, block_priority{ .Level = RightChild.Level, .Error = RightError, .BlockId = RightChild.BlockId });
  }

  return true;
}

static void
WriteBlock(i8 Level, u64 BlockIdx) {
  bitstream* Bs = &BlockStreams[Level];
  if (Size(*Bs) > 0) {
    Flush(Bs);
    FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, Level), "ab");
    fwrite(Bs->Stream.Data, Size(*Bs), 1, Fp);
    fclose(Fp);

    // book-keeping
    if (BlockBytes[Level].size() <= BlockIdx)
      BlockBytes[Level].resize(BlockIdx * 3 / 2 + 1);
    BlockBytes[Level][BlockIdx] = Size(*Bs);
    Rewind(Bs);
  }
}

/* Write each level to a different file */
static void
FlushBlocksToFiles() {
  /* write the resolution tree */
  FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, Params.NLevels), "wb");
  Flush(&BlockStreams[Params.NLevels]);
  fwrite(BlockStreams[Params.NLevels].Stream.Data, Size(BlockStreams[Params.NLevels]), 1, Fp);
  fclose(Fp);

  /* write the level data */
  REQUIRE(BlockStreams.size() == Params.NLevels + 1);
  FOR(i8, L, 0, Params.NLevels) {
    WriteBlock(L, CurrBlocks[L]);
    // write an index consisting of all blocks in the file
    // TODO: compress the index?
    // TODO: if too many blocks have 0 bytes, maybe we can write a sparse index
    FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, L), "ab");
    u64 NBlocks = CurrBlocks[L] + 1;
    FOR(u64, BlockIdx, 0, NBlocks) {
      fwrite(&BlockBytes[L][BlockIdx], sizeof(i64), 1, Fp);
    }
    fwrite(&NBlocks, sizeof(NBlocks), 1, Fp);
    fclose(Fp);
  }
  /* write the meta-data file */
  WriteMetaFile(Params, PRINT("%s.idx", Params.OutFile));
}

INLINE static void
EncodeNode(i8 Level, i64 NodeIdx, i64 M, i64 N) {
  // TODO: use binomial coding
  u64 BlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  printf("level = %d block = %llu\n", Level, BlockIdx);
  if (BlockIdx > CurrBlocks[Level]) { // we have moved to the next block, dump the current block to disk
    WriteBlock(Level, CurrBlocks[Level]);
    CurrBlocks[Level] = BlockIdx;
  }
  bitstream* Bs = &BlockStreams[Level];
  GrowToAccomodate(Bs, 8);
  WriteVarByte(Bs, N);
}

INLINE static void
EncodeRoot(i64 N) {
  InitWrite(&BlockStreams[Params.NLevels], 1024);
  GrowToAccomodate(&BlockStreams[Params.NLevels], 8);
  WriteVarByte(&BlockStreams[Params.NLevels], N);
}

INLINE static void /* encode a resolution node */
EncodeResNode(i64 M, i64 N) {
  // TODO: use binomial coding
  GrowToAccomodate(&BlockStreams[Params.NLevels], 8);
  WriteVarByte(&BlockStreams[Params.NLevels], N);
}

INLINE static void // N is the number of particles under a node
Print(i8 Level, u64 TreeIdx, i64 ResIdx, i64 LvlIdx, i64 ParIdx, i64 N) {
  printf("level = %d tree_idx = %llu res_idx = %lld lvl_idx = %lld par_idx = %lld N = %lld \n", Level, TreeIdx, ResIdx, LvlIdx, ParIdx, N);
  return;
}

static grid
SplitGrid(const grid& Grid, int D, bool RSplit, bool Right) {
  //REQUIRE(IS_EVEN(int(Grid.Dims3[D])));
  grid Out = Grid;
  if (RSplit) { // resolution split
    Out.From3[D] += Right * Out.Stride3[D];
    Out.Dims3[D] *= 0.5;
    Out.Stride3[D] *= 2;
  } else { // spatial split
    Out.Dims3[D] *= 0.5;
    Out.From3[D] += Right * Out.Stride3[D] * Out.Dims3[D];
  }
  return Out;
}

struct q_item {
  i64 Begin, End;
  u64 TreeIdx; // the index in the tree
  i64 ResIdx; // index in the resolution tree
  i64 NodeIdx; // index within its own level (either this is used or the ResIdx is used, not both)
  i64 ParIdx; // particle idx (i.e., number of particles to the left of it)
  grid Grid;
  vec3f Error;
  i8 D;
  i8 Level;
  u8 Height;
  bool RSplit;
};

struct Range {
  u64 From, To;
};

std::vector<bbox> BBoxes; // one bounding box for each particle
std::vector<Range> Ranges; // [level] -> from, to

static void
BuildTreeFineLevels(u8 Height) {
  FOR (i8, L, 0, Params.NLevels) {
  FOR (u64, I, Ranges[L].From, Ranges[L].To) {
    i8 D = (Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z) % Params.NDims;
    vec3f P3 = Particles[I].Pos;
    bbox BBox = BBoxes[I];
    // TODO: use the accuracy and track the block
    FOR (u8, H, 0, Height) {
      bitstream* Bs; // TODO: initialize and/or rewind
      float Half = (BBox.Max[D] - BBox.Min[D]) * 0.5;
      bool Left = (P3[D] - BBox.Min[D]) < Half;
      Write(Bs, Left);
      BBox.Max[D] = BBox.Max[D] - Half * Left;
      BBox.Min[D] = BBox.Min[D] + Half * (1 - Left);
      D = (D + 1) % 3;
    }
  }}
}

// TODO: after blocking, we have a tree of blocks
// TODO: write a routine to decode the blocks
// TODO: write a routine to read from disk and reconstruct the tree/particles
// TODO: write a routine to compute the PSNR of positions
// TODO: what if we have 0 particles? should we stop the resolution divide?
static void
BuildTreeInner(q_item Q, float Accuracy) {
  int GridHeight = Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z;
  std::queue<q_item> Queue;
  Queue.push(Q);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
  while (!Queue.empty()) {
    Q = Queue.front();
    Queue.pop();
    REQUIRE(Q.Height <= Params.Height);
    i64 N = Q.End - Q.Begin;
    assert((N == 1) || IS_EVEN(int(Q.Grid.Dims3[Q.D])));
    i64 Mid = Q.Begin;
    vec3f Error3 = (W3 * Q.Grid.Dims3) / f64(N);
    bool Stop = Error3.x <= Accuracy && Error3.y <= Accuracy;
    if (Params.NDims > 2) Stop = Stop && Error3.z <= Accuracy;
    if (Stop) continue;
    //if (N <= 1) continue;
    if (Q.RSplit) { // resolution split
      auto RPred = [W3, &Q](const particle& P) {
        int Bin = MIN(Params.Dims3[Q.D] - 1, int((P.Pos[Q.D] - Params.BBox.Min[Q.D]) / W3[Q.D]));
        assert(IS_INT(Q.Grid.From3[Q.D]) && IS_INT(Q.Grid.Stride3[Q.D]) && IS_INT(Q.Grid.Dims3[Q.D]));
        REQUIRE((Bin - int(Q.Grid.From3[Q.D])) % int(Q.Grid.Stride3[Q.D]) == 0);
        Bin = (Bin - int(Q.Grid.From3[Q.D])) / int(Q.Grid.Stride3[Q.D]);
        return IS_EVEN(Bin);
      };
      Mid = partition(RANGE(Particles, Q.Begin, Q.End), RPred) - Particles.begin();
    } else { // spatial split
      float S = (Q.Grid.Dims3[Q.D] > 1.5f) * (Q.Grid.Stride3[Q.D] - 1) + 1;
      float M = Params.BBox.Min[Q.D] + W3[Q.D] * (Q.Grid.From3[Q.D] + Q.Grid.Dims3[Q.D] * 0.5f * S);
      auto SPred = [M, &Q](const particle& P) { return P.Pos[Q.D] < M; };
      Mid = partition(RANGE(Particles, Q.Begin, Q.End), SPred) - Particles.begin();
    }
    if (Q.RSplit) {
      EncodeResNode(Q.End - Q.Begin, Mid - Q.Begin);
      Ranges[Q.Level].From = Mid;
      Ranges[Q.Level].To   = Q.End;
      if (Q.Level == 1) {
        Ranges[0].From = Q.Begin;
        Ranges[0].To = Q.End;
      }
    } else {
      EncodeNode(Q.Level - Q.RSplit, Q.RSplit ? Q.NodeIdx : Q.NodeIdx * 2, Q.End - Q.Begin, Mid - Q.Begin);
      printf("%lld\n", Mid - Q.Begin);
      printf("%lld\n", Q.End - Mid);
    }
    if (Q.Height == GridHeight) { // last height
      REQUIRE(N == 1);
      assert(Q.Grid.Dims3.x <= 1 && Q.Grid.Dims3.y <= 1 && Q.Grid.Dims3.z <= 1);
      BBoxes[Q.Begin].Min = Params.BBox.Min + Q.Grid.From3 * W3;
      BBoxes[Q.Begin].Max = BBoxes[Q.Begin].Min + Q.Grid.Dims3 * W3;
    } else {
      //Print(Q.Level - Q.RSplit, Q.TreeIdx * 2 + 1, Q.RSplit ? Q.ResIdx * 2 + 1 : Q.ResIdx, Q.RSplit ? Q.LvlIdx : Q.LvlIdx * 2 + 1, Q.ParIdx, Mid - Q.Begin); // encode only the left child
      if (Q.Height + 1 < (Params.Height) && Q.Begin < Mid) {
        Queue.push(q_item{ .Begin = Q.Begin,
                           .End = Mid,
                           .TreeIdx = Q.TreeIdx * 2,
                           .ResIdx = Q.RSplit ? Q.ResIdx + 2 : Q.ResIdx,
                           .NodeIdx = Q.RSplit ? Q.NodeIdx : Q.NodeIdx * 2,
                           .ParIdx = Q.ParIdx,
                           .Grid = SplitGrid(Q.Grid, Q.D, Q.RSplit, false),
                           .D = i8((Q.D + 1) % Params.NDims),
                           .Level = i8(Q.Level - Q.RSplit),
                           .Height = u8(Q.Height + 1),
                           .RSplit = N > 1 && Q.RSplit && Q.Level > 1 });
      }
      if (Q.Height + 1 < (Params.Height) && Mid < Q.End) {
        Queue.push(q_item{ .Begin = Mid,
                           .End = Q.End,
                           .TreeIdx = Q.TreeIdx * 2 + 1,
                           .ResIdx = Q.RSplit ? Q.ResIdx + 1 : Q.ResIdx,
                           .NodeIdx = Q.RSplit ? Q.NodeIdx : Q.NodeIdx * 2 + 1,
                           .ParIdx = Q.ParIdx + Mid - Q.Begin,
                           .Grid = SplitGrid(Q.Grid, Q.D, Q.RSplit, true),
                           .D = i8((Q.D + 1) % Params.NDims),
                           .Level = Q.Level,
                           .Height = u8(Q.Height + 1),
                           .RSplit = false });
      }
    }
  }
}

/* Return the dimensions of the underlying grid (in terms of power of two) */
static vec3i
ComputeGrid(std::vector<particle>* Particles, const bbox& BBox, i64 Begin, i64 End, i8 D) {
  REQUIRE(Begin < End); // this cannot be a leaf node
  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P) { return P.Pos[D] < Middle; };
  i64 Mid = std::partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left = MCOPY(vec3i(0), [D] = 1), LogDims3Right = MCOPY(vec3i(0), [D] = 1);
  if (Begin + 1 < Mid) {
    LogDims3Left = ComputeGrid(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, (D + 1) % Params.NDims);
    ++LogDims3Left[D];
  }
  if (Mid + 1 < End) {
    LogDims3Right = ComputeGrid(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, (D + 1) % Params.NDims);
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

#define EXIT_ERROR(Msg) { fprintf(stderr, Msg); exit(1); }

int
main(int Argc, cstr* Argv) {
  doctest::Context context(Argc, Argv);
  context.setAsDefaultForAssertsOutOfTestCases();
  context.setAssertHandler(Handler);
  cstr ErrorMsg = "Usage: \n"
                  "  to encode: .exe particle_file.xyz --action encode --ndims 3 --nlevels 4 --height 6 --block 2 --out output\n"
                  "  to decode: .exe compressed_file --action decode --out particle_file.xyz";
  cstr Action = nullptr;
  if (!OptVal(Argc, Argv, "--action", &Action)) EXIT_ERROR(ErrorMsg);
  if (strcmp("encode", Action) == 0) Params.Action = action::Encode;
  else if (strcmp("decode", Action) == 0) Params.Action = action::Decode;
  else EXIT_ERROR(ErrorMsg);

  if (Params.Action == action::Encode) {
    if (!OptVal(Argc, Argv, "--name", &Params.OutFile)) EXIT_ERROR("missing --name");
    sprintf(Params.Name, "%s", Params.OutFile);
    if (!OptVal(Argc, Argv, "--ndims", &Params.NDims)) EXIT_ERROR("missin --ndims");
    if (!OptVal(Argc, Argv, "--nlevels", &Params.NLevels)) EXIT_ERROR("missing --nlevels");
    if (!OptVal(Argc, Argv, "--height", &Params.Height)) {
      if (!OptVal(Argc, Argv, "--accuracy", &Params.Accuracy))
        EXIT_ERROR("missing --height");
    }
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
//    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    if (!OptVal(Argc, Argv, "--block", &Params.BlockBits)) EXIT_ERROR("missing --block");
    Particles = ReadXYZ(Params.InFile);
    Params.NParticles = Particles.size();
    printf("number of particles = %zu\n", Particles.size());
    Params.BBox = ComputeBoundingBox(Particles);
    Params.LogDims3 = ComputeGrid(&Particles, Params.BBox, 0, Particles.size(), 0);
    Params.Dims3 = vec3i(1 << Params.LogDims3.x, 1 << Params.LogDims3.y, 1 << Params.LogDims3.z);
    grid Grid{.From3 = vec3f(0), .Dims3 = vec3f(Params.Dims3), .Stride3 = vec3f(1)};
    printf("bounding box = (" PRIvec3f ") - (" PRIvec3f ")\n", EXPvec3(Params.BBox.Min), EXPvec3(Params.BBox.Max));
    printf("log dims 3 = " PRIvec3i "\n", EXPvec3(Params.LogDims3));
    //Print(NResLevels - 1, 0, 0, 0, 0, Particles.size());
    BlockStreams.resize(Params.NLevels + 1);
    CurrBlocks.resize(Params.NLevels, 0);
    BlockBytes.resize(Params.NLevels);
    Ranges.resize(Params.NLevels);
    BBoxes.resize(Params.NParticles);
    EncodeRoot(Particles.size());
    BuildTreeInner(q_item{ .Begin = 0,
                           .End = (i64)Particles.size(),
                           .TreeIdx = 1,
                           .ResIdx = 0,
                           .NodeIdx = 1,
                           .ParIdx = 0,
                           .Grid = Grid,
                           .Level = i8(Params.NLevels - 1),
                           .Height = 0,
                           .RSplit = Params.NLevels > 1 }, Params.Accuracy);
    FlushBlocksToFiles();
  } else if (Params.Action == action::Decode) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    ReadMetaFile(Params.InFile);
    BlockStreams.resize(Params.NLevels + 1);
    Blocks.resize(Params.NLevels + 1);
    Blocks[Params.NLevels].resize(1);
    Heap.insert(block_data{ .Level = Params.NLevels, .BlockId = 0 },
                block_priority{ .Level = Params.NLevels, .Error = 0, .BlockId = 0 });
    bool Continue = true;
    while (Continue) {
      Continue = RefineByLevel();
    }
  }

  //RandomLevels(P, &Particles);
}

