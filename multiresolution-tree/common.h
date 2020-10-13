#pragma once

#define _CRT_SECURE_NO_WARNINGS
#undef min
#undef max
#undef near
#undef far

#include "doctest.h"
#include "heap.h"
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
#include <optional>

#define POW2(X) (1ull << (X))

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

inline mallocator& Mallocator() {
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

template <typename t> inline void
AllocBufT(buffer_t<t>* Buf, i64 Size, allocator* Alloc) {
  buffer RawBuf;
  AllocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  Buf->Data = (t*)RawBuf.Data;
  Buf->Size = Size;
  Buf->Alloc = Alloc;
}

template <typename t> inline void
CallocBufT(buffer_t<t>* Buf, i64 Size, allocator* Alloc) {
  buffer RawBuf;
  CallocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  Buf->Data = (t*)RawBuf.Data;
  Buf->Size = Size;
  Buf->Alloc = Alloc;
}

template <typename t> inline void
DeallocBufT(buffer_t<t>* Buf) {
  buffer RawBuf{(byte*)Buf->Data, i64(Buf->Size * sizeof(t)), Buf->Alloc};
  DeallocBuf(&RawBuf);
  Buf->Data  = nullptr;
  Buf->Size  = 0;
  Buf->Alloc = nullptr;
}

template <typename t> inline void
AllocPtr(t** Ptr, i64 Size, allocator* Alloc = &Mallocator()) {
  buffer RawBuf;
  AllocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  *Ptr = (t*)RawBuf.Data;
}

template <typename t> inline void
CallocPtr(t** Ptr, i64 Size, allocator* Alloc = &Mallocator()) {
  buffer RawBuf;
  CallocBuf(&RawBuf, i64(Size * sizeof(t)), Alloc);
  *Ptr = (t*)RawBuf.Data;
}

template <typename t> inline void
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
  if (NewCapacity > OriginalCapacity) {
    IncreaseCapacity(Bs, NewCapacity);
  }
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

inline i64
MemCopy(const buffer& Src, buffer* Dst) {
  assert(Dst->Data);
  assert(Src.Data || Src.Bytes == 0);
  assert(Dst->Bytes >= Src.Bytes);
  memcpy(Dst->Data, Src.Data, size_t(Src.Bytes));
  return Src.Bytes;
}

inline i64
MemCopy(const buffer& Src, buffer* Dst, u64 Bytes) {
  assert(Dst->Data);
  assert(Src.Data || Src.Bytes == 0);
  assert(Dst->Bytes >= Src.Bytes);
  memcpy(Dst->Data, Src.Data, size_t(Bytes));
  return Bytes;
}

inline buffer
operator+(const buffer& Buf, i64 Bytes) {
  return buffer{ Buf.Data + Bytes, Buf.Bytes - Bytes };
}

inline void
ZeroBuf(buffer* Buf) {
  assert(Buf->Data);
  memset(Buf->Data, 0, size_t(Buf->Bytes));
}

template <typename t> inline void
ZeroBufT(buffer_t<t>* Buf) {
  assert(Buf->Data);
  memset(Buf->Data, 0, Buf->Size * sizeof(t));
}

inline void
AllocBuf(buffer* Buf, i64 Bytes, allocator* Alloc) {
  Alloc->Alloc(Buf, Bytes);
}

inline void
CallocBuf(buffer* Buf, i64 Bytes, allocator* Alloc) {
  assert(!Buf->Data || Buf->Bytes == 0);
  if (Alloc == &Mallocator()) {
    Buf->Data = (byte*)calloc(size_t(Bytes), 1);
  } else {
    AllocBuf(Buf, Bytes, Alloc);
    ZeroBuf(Buf);
  }
  Buf->Bytes = Bytes;
  Buf->Alloc = Alloc;
}

inline void
DeallocBuf(buffer* Buf) {
  assert(Buf->Alloc);
  Buf->Alloc->Dealloc(Buf);
}

inline bool
mallocator::Alloc(buffer* Buf, i64 Bytes) {
  assert(!Buf->Data || Buf->Bytes == 0);
  Buf->Data = (byte*)malloc(size_t(Bytes));
  Buf->Bytes = Bytes;
  Buf->Alloc = this;
  return true;
}

inline void
mallocator::Dealloc(buffer* Buf) {
  free(Buf->Data);
  Buf->Data = nullptr;
  Buf->Bytes = 0;
  Buf->Alloc = nullptr;
}

inline void
mallocator::DeallocAll() { /* empty */ }

inline linear_allocator::
linear_allocator() = default;

inline linear_allocator::
linear_allocator(const buffer & Buf) : Block(Buf) {}

inline bool linear_allocator::
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

inline void linear_allocator::
Dealloc(buffer * Buf) {
  if (Buf->Data + Buf->Bytes == Block.Data + CurrentBytes) {
    Buf->Data = nullptr;
    Buf->Bytes = 0;
    Buf->Alloc = nullptr;
    CurrentBytes -= Buf->Bytes;
  }
}

inline void linear_allocator::
DeallocAll() {
  CurrentBytes = 0;
}

inline bool linear_allocator::
Own(const buffer & Buf) const {
  return Block.Data <= Buf.Data && Buf.Data < Block.Data + CurrentBytes;
}

inline free_list_allocator::
free_list_allocator() = default;

inline free_list_allocator::
free_list_allocator(i64 MinBytesIn, i64 MaxBytesIn, allocator * ParentIn)
  : MinBytes(MinBytesIn)
  , MaxBytes(MaxBytesIn)
  , Parent(ParentIn) {}

inline free_list_allocator::
free_list_allocator(i64 Bytes, allocator * ParentIn)
  : free_list_allocator(Bytes, Bytes, ParentIn) {}

inline bool free_list_allocator::
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

inline void free_list_allocator::
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
inline void free_list_allocator::
DeallocAll() {
  assert(Parent);
  while (Head) {
    node* Next = Head->Next;
    buffer Buf((byte*)Head, MaxBytes, Parent);
    Parent->Dealloc(&Buf);
    Head = Next;
  }
}

inline fallback_allocator::
fallback_allocator() = default;

inline fallback_allocator::
fallback_allocator(owning_allocator * PrimaryIn, allocator * SecondaryIn)
  : Primary(PrimaryIn)
  , Secondary(SecondaryIn) {}

inline bool fallback_allocator::
Alloc(buffer * Buf, i64 Size) {
  bool Success = Primary->Alloc(Buf, Size);
  return Success ? Success : Secondary->Alloc(Buf, Size);
}

inline void fallback_allocator::
Dealloc(buffer * Buf) {
  if (Primary->Own(*Buf))
    return Primary->Dealloc(Buf);
  Secondary->Dealloc(Buf);
}

inline void fallback_allocator::
DeallocAll() {
  Primary->DeallocAll();
  Secondary->DeallocAll();
}

inline void
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

inline bool
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

inline bool
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
inline bool
OptVal(int NArgs, cstr* Args, cstr Opt, cstr* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      *Val = Args[I + 1];
      return true;
    }
  }
  return false;
}

inline bool
OptVal(int NArgs, cstr* Args, cstr Opt, int* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0)
      return ToInt(Args[I + 1], Val);
  }
  return false;
}

inline bool
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

inline bool
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

inline bool
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

inline bool
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

inline bool
OptVal(int NArgs, cstr* Args, cstr Opt, vec2i* Val) {
  for (int I = 0; I + 2 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0) {
      return ToInt(Args[I + 1], &Val->x) &&
        ToInt(Args[I + 2], &Val->y);
    }
  }
  return false;
}

inline bool
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

inline bool
OptVal(int NArgs, cstr* Args, cstr Opt, f64* Val) {
  for (int I = 0; I + 1 < NArgs; ++I) {
    if (strncmp(Args[I], Opt, 32) == 0)
      return ToDouble(Args[I + 1], Val);
  }
  return false;
}

inline bool
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


inline bool
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
    ::InitWrite(&BitStream, Bytes);
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

#define erfinv_a3 -0.140543331
#define erfinv_a2 0.914624893
#define erfinv_a1 -1.645349621
#define erfinv_a0 0.886226899

#define erfinv_b4 0.012229801
#define erfinv_b3 -0.329097515
#define erfinv_b2 1.442710462
#define erfinv_b1 -2.118377725
#define erfinv_b0 1

#define erfinv_c3 1.641345311
#define erfinv_c2 3.429567803
#define erfinv_c1 -1.62490649
#define erfinv_c0 -1.970840454

#define erfinv_d2 1.637067800
#define erfinv_d1 3.543889200
#define erfinv_d0 1
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944

inline double erfinv (double x) {
  double x2, r, y;
  int  sign_x;

  if (x < -1 || x > 1)
    return NAN;

  if (x == 0)
    return 0;

  if (x > 0)
    sign_x = 1;
  else {
    sign_x = -1;
    x = -x;
  }

  if (x <= 0.7) {
    x2 = x * x;
    r =
      x * (((erfinv_a3 * x2 + erfinv_a2) * x2 + erfinv_a1) * x2 + erfinv_a0);
    r /= (((erfinv_b4 * x2 + erfinv_b3) * x2 + erfinv_b2) * x2 +
      erfinv_b1) * x2 + erfinv_b0;
  }
  else {
    y = sqrt (-log ((1 - x) / 2));
    r = (((erfinv_c3 * y + erfinv_c2) * y + erfinv_c1) * y + erfinv_c0);
    r /= ((erfinv_d2 * y + erfinv_d1) * y + erfinv_d0);
  }

  r = r * sign_x;
  x = x * sign_x;

  r -= (erf (r) - x) / (2 / sqrt (M_PI) * exp (-r * r));
  r -= (erf (r) - x) / (2 / sqrt (M_PI) * exp (-r * r));

  return r;
}

#undef erfinv_a3
#undef erfinv_a2
#undef erfinv_a1
#undef erfinv_a0

#undef erfinv_b4
#undef erfinv_b3
#undef erfinv_b2
#undef erfinv_b1
#undef erfinv_b0

#undef erfinv_c3
#undef erfinv_c2
#undef erfinv_c1
#undef erfinv_c0

#undef erfinv_d2
#undef erfinv_d1
#undef erfinv_d0

// TODO: count the number of bits per level

const inline double sqrt2 = sqrt(2.0);
/* The Gaussian CDF. m = mean, s = standard deviation */
INLINE double
F(double m, double s, double x) {
  return 0.5 * std::erfc((m - x) / (s * sqrt2));
}

/* The inverse Gaussian CDF. m = mean, s = standard deviation */
INLINE double
Finv(double m, double s, double y) {
  return m + s * (erfinv(2 * y - 1) * sqrt2);
}

/* Reverse the bits in the input */
inline uint
BitReverse(uint a) {
  uint t;
  a = (a << 15) | (a >> 17);
  t = (a ^ (a >> 10)) & 0x003f801f;
  a = (t + (t << 10)) ^ a;
  t = (a ^ (a >>  4)) & 0x0e038421;
  a = (t + (t <<  4)) ^ a;
  t = (a ^ (a >>  2)) & 0x22488842;
  a = (t + (t <<  2)) ^ a;
  return a;
}

/* v is from 0 to n-1 */
inline void
EncodeCenteredMinimal(u32 v, u32 n, bitstream* Bs) {
  assert(n > 0);
  assert(v < n);
  if (n == 2) {
    Write(Bs, v == 1);
    return;
  }
  u32 l1 = Msb(n);
  u32 l2 = ((1 << l1) == n) ? l1 : l1 + 1;
  u32 d = (1 << l2) - n;
  u32 m = (n - d) / 2;
  if (v < m) {
    v = BitReverse(v);
    v >>= sizeof(v) * 8 - l2;
    Write(Bs, v, l2);
  } else if (v >= m + d) {
    v = BitReverse(v - d);
    v >>= sizeof(v) * 8 - l2;
    Write(Bs, v, l2);
  } else { // middle
    v = BitReverse(v);
    v >>= sizeof(v) * 8 - l1;
    Write(Bs, v, l1);
  }
}

inline u32
DecodeCenteredMinimal(u32 n, bitstream* Bs) {
  assert(n > 0);
  if (n == 2) {
    return (u32)Read(Bs);
  }
  u32 l1 = Msb(n);
  u32 l2 = ((1 << l1) == n) ? l1 : l1 + 1;
  u32 d = (1 << l2) - n;
  u32 m = (n - d) / 2;
  Refill(Bs); // TODO: minimize the number of refill
  u32 v = (u32)Peek(Bs, l2);
  v <<= sizeof(v) * 8 - l2;
  v = BitReverse(v);
  if (v < m) {
    Consume(Bs, l2);
    return v;
  } else if (v < 2 * m) {
    Consume(Bs, l2);
    return v + d;
  } else {
    Consume(Bs, l1);
    return v >> 1;
  }
}

using cdf = std::vector<u32>;
using cdf_table = std::vector<cdf>;

/* Generate the Pascal triangle */
inline cdf_table 
pascal_triangle(int n) {
  cdf_table triangle(n + 1);
  for (int i = 0; i <= n; ++i)
    triangle[i] = std::vector<u32>(i + 1);

  for (int i = 0; i <= n; ++i) {
    triangle[i][0] = 1;
    for (int j = 1; j < i; ++j)
      triangle[i][j] = triangle[i-1][j-1] + triangle[i-1][j];
    triangle[i][i] = 1;
  }

  return triangle;
}

/* Create a probability table for small N */
inline cdf_table
CreateBinomialTable(int N) {
  auto table = pascal_triangle(N);
  for (int n = 0; n <= N; ++n)
    for (int k = 1; k <= n; ++k)
      table[n][k] += table[n][k-1];
  return table;
}

inline void
EncodeBinomialSmallRange(int n, int v,  const cdf& CdfTable, arithmetic_coder<>* Coder) {
  assert(v >= 0 && v <= n);
  u32 lo = v == 0 ? 0 : CdfTable[v - 1];
  u32 hi = CdfTable[v];
  u32 scale = 1 << n;
  prob<u32> prob{lo, hi, scale};
  Coder->Encode(prob);
}

inline int
DecodeBinomialSmallRange(int n, const cdf& CdfTable, arithmetic_coder<>* Coder) {
  size_t v = Coder->Decode(CdfTable);
  assert(v <= n);
  return (int)v;
}

constexpr inline int cutoff1 = 32; // cannot be bigger than 32 else we will have overflow
constexpr inline int cutoff2 = 0; // to switch over to uniform encoding (doesn't seem to make a big difference in compression rate, but may make a difference in speed)

/* The inverse of encode */
// TODO: refactor to put part the logic of this function to the decode function
inline int
DecodeRange(
  double m, double s, double a, double b,
  const cdf_table& CdfTable, bitstream* Bs, arithmetic_coder<>* Coder) 
{
  assert(a <= b);
  bool first = true;
  while (true) {
    int beg = (int)std::ceil(a);
    int end = (int)std::floor(b);
    if (beg == end)
      return beg; // no need to write any bit
    int n = end - beg + 1;
    if (first && n <= cutoff1)
      return DecodeBinomialSmallRange(n - 1, CdfTable[n - 1], Coder);
    if (!first && n <= cutoff2)
      return beg + DecodeCenteredMinimal(n, Bs);
    /* compute F(a) and F(b) */
    double fa = F(m, s, a);
    double fb = F(m, s, b);
    // TODO: what if fa==fb
    /* compute F^-1((fa+fb)/2) */
    double mid = Finv(m, s, (fa + fb) * 0.5);
    if (mid < a || mid > b) // mid can be infinity when (fa+fb) == 0
      mid = a;
    if (a == mid || b == mid)
      return beg + DecodeCenteredMinimal(n, Bs);
    assert(a <= mid && mid <= b);

    auto bit = Read(Bs);
    if (bit == 0) b = std::floor(mid);
    else          a = std::ceil(mid);

    first = false;
  }
}

/* Assuming a Gaussian(m, s), and a range [a, b] (0<=a<=b<=N), and c (a<=c<=b), partition [a,b]
into two bins of equal probability */
inline void
EncodeRange(double m, double s,
  double a, double b, double c,
  const cdf_table& CdfTable, bitstream* Bs, arithmetic_coder<>* Coder)
{
  assert(a <= b);
  bool first = true;

  /* comment out the below to use uniform encoding instead of gaussian distribution */
  //int beg = cast(int)ceil(a);
  //int end = cast(int)floor(b);
  //int v = cast(int)c-beg;
  //int n = end - beg + 1; // v can be from 0 to n-1
  //if (end-beg+1 <= cutoff)
  //  return encode_binomial_small_range(n-1, v, CdfTable[n-1], coder);
  //else
  //  return encode_centered_minimal(v, n, bs);

  while (true) {
    int beg = (int)std::ceil(a);
    int end = (int)std::floor(b);
    if (beg == end)
      return; // no need to write any bit
    int n = end - beg + 1; // v can be from 0 to n-1
    int v = int(c - beg);
    if (first && n <= cutoff1)
      return EncodeBinomialSmallRange(n - 1, v, CdfTable[n - 1], Coder);
    if (!first && n <= cutoff2)
      return EncodeCenteredMinimal(v, n, Bs);
    /* compute F(a) and F(b) */
    double fa = F(m, s, a);
    double fb = F(m, s, b);
    /* compute F^-1((fa+fb)/2) */
    double mid = Finv(m, s, (fa + fb) * 0.5);
    //assert(mid == mid);
    if (mid < a || mid > b) // mid can be infinity when (fa+fb) == 0
      mid = a;
    if (a == mid || b == mid || mid != mid)
      return EncodeCenteredMinimal(v, n, Bs);
    assert(a <= mid && mid <= b);
    if (c < mid) {
      Write(Bs, 0);
      b = floor(mid);
    } else { // c >= mid
      Write(Bs, 1);
      a = ceil(mid);
    }
    first = false;
  }
}

struct empty_struct { };

struct bbox { vec3f Min, Max; };
INLINE vec3f Extent(const bbox& BBox) { return BBox.Max - BBox.Min; }

enum node_type { Root, Inner };

struct particle {
  vec3f Pos; // position
  //u64 Code = 0; //
};

struct grid {
  vec3f From3, Dims3, Stride3;
};

enum split_type { ResolutionSplit, SpatialSplit };
enum side { Left, Right };
enum class action : int { Encode, Decode, Error };

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
  split_type SplitType;
};

struct params {
  char Name[64];
  vec2i Version = vec2i(1, 0);
  int NDims = 3;
  cstr InFile;
  cstr OutFile;
  int BlockBits = 18; // every 2^15 voxels become one block
  i8 NLevels = 3;
  u8 MaxHeight = 255; // height of the full tree
  action Action = action::Encode;
  i64 NParticles;
  float Accuracy = 0;
  bbox BBox;
  vec3i LogDims3;
  vec3i BlockDims3; // TODO: compute this
  u8 BaseHeight;
  vec3i Dims3;
  int MaxNBlocks = INT_MAX;
  i8 MaxLevel = 127;
  int MaxParticleSubSampling = 0;
  bool NoRefinement = false;
};

inline grid
SplitGrid(const grid& Grid, int D, split_type SplitType, side Side) {
  grid Out = Grid;
  if (SplitType == ResolutionSplit) {
    Out.From3[D] += (Side == Right) * Out.Stride3[D];
    Out.Dims3[D] *= 0.5;
    Out.Stride3[D] *= 2;
  } else { // spatial split
    Out.Dims3[D] *= 0.5;
    Out.From3[D] += (Side == Right) * Out.Stride3[D] * Out.Dims3[D];
  }
  return Out;
}

inline params Params;
struct ref_block {
  i8 Level = 0;
  u64 BlockId = 0;
};
struct block_meta {
  i64 Size = 0;
  u64 BlockId = 0;
};
INLINE bool operator<(const block_meta& Lhs, const block_meta& Rhs) {
  return Lhs.BlockId < Rhs.BlockId;
}

struct block {
  std::vector<i64> Nodes; // used when level <= BaseHeight
//  bitset BitSet = bitset{ nullptr, 0 }; // used when level > BaseHeight
  bitstream Bs;
  int NParticles = 0; // only used when level > BaseHeight (to complement BitSet)
  block() { Nodes.resize(POW2(Params.BlockBits)); }
  block(bitstream* Bs) { 
    Nodes.resize(Size(Bs->Stream) / sizeof(Nodes[0]) + 1);
    memcpy(Nodes.data(), Bs->Stream.Data, Size(Bs->Stream));
    this->Bs.Stream.Data = (byte*)Nodes.data();
    this->Bs.Stream.Bytes = Bs->Stream.Bytes;
    InitRead(&this->Bs, this->Bs.Stream); 
  }
};
using block_table = std::vector<std::vector<block>>; // [level] -> [block id] -> block data
inline block_table Blocks;
inline std::vector<particle> Particles;
inline std::vector<bitstream> BlockStreams; // [level] -> bitstream (of the current block)
inline std::vector<bitstream> RefBlockStreams; // [height] -> bitstream (of the current block)
inline std::vector<u64> CurrBlocks; // [level] -> current block id
inline std::vector<ref_block> CurrRefBlocks; // [height] -> current refinement block
inline std::vector<std::vector<block_meta>> BlockOffsets; // [level] -> [block id] -> block offset
inline int MaxBlockSize = 0; // max block size
inline std::vector<byte> Padding;
inline std::vector<std::vector<block_meta>> BlockBytes; // [level] -> [block id] -> block size
inline i64 NBlocksWritten = 0;

#define NODE_TO_BLOCK_INDEX(Idx) ((Idx) >> (Params.BlockBits))
#define NODE_INDEX_IN_BLOCK(Idx) ((Idx) & (POW2(Params.BlockBits) - 1))
#define LEVEL_TO_HEIGHT(Level) ((Params.NLevels - (Level)) - ((Level) == 0))
#define NUM_BLOCKS_AT_LEAF(Level) POW2(MAX(0, Params.BaseHeight - LEVEL_TO_HEIGHT(Level) - Params.BlockBits))
#define NUM_NODES_AT_LEAF(Level) POW2(MAX(0, Params.BaseHeight - LEVEL_TO_HEIGHT(Level)))
#define LEVEL_TO_NODE(Level) (((Level) > 0) + (Params.NLevels - 1 - (Level)) * 2)

/* tree block (including refinement block) */
inline void
WriteBlock(bitstream* Bs, i8 Level, u64 BlockIdx) {
//  printf("--------- writing level %d block %llu\n", Level, BlockIdx);
  if (Size(*Bs) > 0) {
    Flush(Bs);
    FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, Level), "ab");
    fwrite(Bs->Stream.Data, Size(*Bs), 1, Fp);
    fclose(Fp);

    // book-keeping
    BlockBytes[Level].push_back(block_meta{.Size = Size(*Bs), .BlockId = BlockIdx});
    MaxBlockSize = MAX(MaxBlockSize, (int)Size(*Bs));
    Rewind(Bs);
    ++NBlocksWritten;
  }
}

inline void
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
  fprintf(Fp, "    (height %d)\n", Params.MaxHeight);
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

struct q_item_new {
  i64 Begin, End;
  i64 Idx; // index in the tree
  grid Grid;
  vec3f Error;
  i8 D; // splitting plane
  u8 Height;
};

struct cosmo_header {
  int np_local; // number of particles
  float a, t, tau;
  int nts;
  float dt_f_acc, dt_pp_acc, dt_c_acc;
  int cur_checkpoint, cur_projection, cur_halofind;
  float massp;
};

inline std::vector<particle>
ReadCosmo(cstr FileName) {
  auto Fp = fopen(FileName, "rb");
  cosmo_header Header;
  ReadPOD(Fp, &Header);
  printf("number of particles = %d\n", Header.np_local);
  std::vector<particle> Particles(Header.np_local);
  vec3f Vel; // velocity
  FOR_EACH(P, Particles) {
    ReadPOD(Fp, &P->Pos);
    ReadPOD(Fp, &Vel); // dummy
  }
  fclose(Fp);
  return Particles;
}

struct vtu_header {
  u32 pad3;
  u32 size;
  u32 pad1;
  u32 step;
  u32 pad2;
  f32 time;
};

inline std::vector<particle>
ReadVtu(cstr FileName) {
  auto Fp = fopen(FileName, "rb");
  std::vector<particle> Particles;
  vtu_header Header;
  int MagicOffset = 4072;
  fseek(Fp, MagicOffset, SEEK_SET);
  ReadPOD(Fp, &Header);
  //auto size = header[0].size;
  //writeln(header[0].size);
  Particles.resize(Header.size);
  fseek(Fp, 4, SEEK_CUR);
  FOR_EACH(P, Particles) {
    fread(&P->Pos, sizeof(*P), 1, Fp);
  }
  //fp.seek(4, SEEK_CUR);
  //fp.rawRead(particles.velocity[0]);
  //fp.seek(4, SEEK_CUR);
  //fp.rawRead(particles.concentration[0]);
  return Particles;
}

/* Read all particles from a XYZ file */
inline std::vector<particle>
ReadXYZ(cstr FileName) {
  FILE* Fp = fopen(FileName, "r");
  std::vector<particle> Particles;

  char Line[256];
  fgets(Line, sizeof(Line), Fp);
  u32 NParticles; sscanf(Line, "%" PRIu32, &NParticles);
  Particles.resize(NParticles);
  fgets(Line, sizeof(Line), Fp); // dummy second line
  FOR(int, I, 0, NParticles) {
    fgets(Line, sizeof(Line), Fp);
    vec3f P3; 
    char C[8];
    sscanf(Line, "%s %f %f %f", C, &P3.x, &P3.y, &P3.z);
    Particles[I].Pos = P3;
  }
  return Particles;
}

template <typename t> inline void
WriteXYZ(cstr FileName, t Begin, t End) {
  FILE* Fp = fopen(FileName, "w");
  auto NParticles = End - Begin;
  fprintf(Fp, "%zu\n", NParticles);
  fprintf(Fp, "dummy\n");
  FOR_EACH (P3, Begin, End) {
    fprintf(Fp, "C %f %f %f\n", P3->Pos.x, P3->Pos.y, P3->Pos.z);
  }
  fclose(Fp);
}

inline char DimsStr[128] = {};

template <node_type R>
struct tree {
  tree<Inner>* Left = nullptr;
  tree<Inner>* Right = nullptr;
  i64 Begin = 0, End = 0;
  using bbox_t = std::conditional_t<R == Root, bbox, empty_struct>;
  [[no_unique_address]] bbox_t BBox = bbox_t();
};

struct particle_cell {
  i64 ParticleId = 0;
  i8 Count = 0;
};

#define ROW3_64(X, Y, Z) ((Z) * 64 * 64 + (Y) * 64 + (X))
#define ROW3(Nx, Ny, Nz, X, Y, Z) ((Z) * 64 * 64 + (Y) * 64 + (X))
#define ROW2_64(X, Y) ((Y) * 64 + (X))

#define LH_IDX(Level, Depth) ((Level) * Params.BaseHeight + (Depth))


