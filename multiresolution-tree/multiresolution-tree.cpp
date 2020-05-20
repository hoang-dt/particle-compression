// multiresolution-tree.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// TODO: better memory allocation (to put the leaves on the same memory block)

#define _CRT_SECURE_NO_WARNINGS

#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#include <exception>
#include "doctest.h"
#include "linalg.h"
#include <algorithm>
#include <iostream>
#include <inttypes.h>
#include <random>
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
#define FOR_EACH(It, Container) for (auto It = begin(Container); It != end(Container); ++It)
#define MAX(A, B) (B) < (A) ? (A) : (B)
#define MIN(A, B) (A) < (B) ? (A) : (B)

#if defined(_MSC_VER)
#define INLINE __forceinline
#elif defined(__clang__) || defined(__GNUC__)
#define INLINE inline __attribute__((__always_inline__))
#endif

using namespace linalg::aliases;
using namespace std;

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

template <typename t>
struct range {
  t Begin, End;
  INLINE i64 size() const { return End - Begin; }
  INLINE t  begin() const { return Begin; }
  INLINE t    end() const { return End  ; }
};

#define RANGE(...) MACRO_OVERLOAD(RANGE, __VA_ARGS__)
#define RANGE_0()
#define RANGE_1(Container) range<decltype(begin(Container))>{ begin(Container), end(Container) }
#define RANGE_2(Begin, End) range<decltype(Begin)>{ Begin, End }
#define RANGE_3(Container, Begin, End) range<decltype(begin(Container))>{ begin(Container) + Begin, begin(Container) + End }

struct empty_struct { };

struct bbox { float3 Min, Max; };

enum node_type { Root, Inner };

struct particle {
  float3 Pos; // position
  u64 Code; // 
};

template <node_type R>
struct kd_tree {
  union {
    struct {
      kd_tree<Inner>* Left;
      kd_tree<Inner>* Right;
    };
    u64 Bits; // only stored at the leaf
  };
  u64 Begin = 0;
  u64 End = 0;
  using bbox_t = conditional_t<R == Root, bbox, empty_struct>;
  using vec_float3_t = conditional_t<R == Root, vector<particle>*, empty_struct>;
  [[no_unique_address]] bbox_t BBox = bbox_t();
  [[no_unique_address]] vec_float3_t Particles = vec_float3_t();
};

/* Read all particles from a XYZ file */
static vector<particle>
ReadXYZ(cstr FileName) {
  FILE* Fp = fopen(FileName, "r");
  vector<particle> Particles;

  char Line[256];
  fgets(Line, sizeof(Line), Fp);
  u32 NParticles; sscanf(Line, "%" PRIu32, &NParticles);
  Particles.resize(NParticles);
  fgets(Line, sizeof(Line), Fp); // dummy second line
  FOR (int, I, 0, NParticles) {
    fgets(Line, sizeof(Line), Fp);
    float3 P3; char C;
    sscanf(Line, "%c %f %f %f", &C, &P3.x, &P3.y, &P3.z);
    Particles[I].Pos = P3;
  }
  return Particles;
}

template <typename t> static void
WriteXYZ(cstr FileName, const range<t>& Particles) {
  FILE* Fp = fopen(FileName, "w");
  auto NParticles = Particles.size();
  fprintf(Fp, "%zu\n", NParticles);
  fprintf(Fp, "dummy\n");
  FOR_EACH (P3, Particles) {
    fprintf(Fp, "C %f %f %f\n", P3->x, P3->y, P3->z);
  }
  fclose(Fp);
}

static bbox
ComputeBoundingBox(const vector<particle>& Particles) {
  REQUIRE(!Particles.empty());
  bbox BBox;
  BBox.Min = BBox.Max = Particles[0].Pos;
  FOR_EACH (P3, Particles) {
    BBox.Min = float3(MIN(BBox.Min.x, P3->Pos.x), MIN(BBox.Min.y, P3->Pos.y), MIN(BBox.Min.z, P3->Pos.z));
    BBox.Max = float3(MAX(BBox.Max.x, P3->Pos.x), MAX(BBox.Max.y, P3->Pos.y), MAX(BBox.Max.z, P3->Pos.z));
  }
  return BBox;
}

// First 4 bits = Level
// Last 60 bits = block id of particle
#define BLOCK_ID(Level, ParticleId, BlockBits) ((u64(Level) << 60) + ((ParticleId) >> ((Level) + (BlockBits))))
#define LEVEL(BlockId) ((BlockId) >> 60)

unordered_map<u64, vector<float3>> ParticlesLODs;
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

using particles = vector<unordered_map<u64, vector<float3>>>; // [level] -> [blocks of particles]
static void
Refine(particles* Particles) {

}

template <node_type R> static void
BuildTreeLeaf(const params& P, kd_tree<R>* CurrNode, const kd_tree<Root>* RootNode, bbox BBox, i64 Begin, i64 End, u64 BitPath, i64 Path) {
  REQUIRE(Begin + 1 == End);
  CurrNode->Begin = Begin;
  CurrNode->End = End;
  const float3& P3 = (*RootNode->Particles)[Begin].Pos;
  while (Path) {
    int D = Path % 3;
    Path = Path / 3;
    auto Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
    bool Left = P3[D] < Middle;
    BitPath = (BitPath << 1) + Left;
    if (Left) BBox.Max[D] = Middle; else BBox.Min[D] = Middle;
  }
  u64 Level = Lsb(BitPath);
  u64 BlockId = BLOCK_ID(Level, BitPath, P.BlockBits);
  ParticlesLODs[BlockId].reserve(128);
  ParticlesLODs[BlockId].push_back(P3);
}

template <node_type R> static void
BuildTreeInner(const params& P, kd_tree<R>* CurrNode, kd_tree<Root>* RootNode, const bbox BBox, i64 Begin, i64 End, u64 BitPath, i64 Path) {
  REQUIRE(Begin < End); // this cannot be a leaf node
  CurrNode->Begin = Begin;
  CurrNode->End = End;
  CurrNode->Left = CurrNode->Right = nullptr;
  int D = Path % 3;
  Path = Path / 3;
  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P3) { return P3.Pos[D] < Middle; };
  auto Right = partition(RootNode->Particles->begin() + Begin, RootNode->Particles->begin() + End, Pred);
  i64 LeftSize = Right - (RootNode->Particles->begin() + Begin);
  u64 PathLeft = 0, PathRight = 0;
  if (LeftSize > 0) {
    CurrNode->Left = new kd_tree<Inner>();
    auto BBoxLeft = BBox;
    BBoxLeft.Max[D] = Middle;
    BitPath <<= 1;
    if (LeftSize == 1) { // leaf
      BuildTreeLeaf(P, CurrNode->Left, RootNode, BBoxLeft, Begin, Begin + LeftSize, BitPath, Path);
    } else { // recurse on the left
      BuildTreeInner(P, CurrNode->Left, RootNode, BBoxLeft, Begin, Begin + LeftSize, BitPath, Path);
    }
  }
  if (Begin + LeftSize < End) {
    CurrNode->Right = new kd_tree<Inner>();
    auto BBoxRight = BBox;
    BBoxRight.Min[D] = Middle;
    BitPath = (BitPath << 1) + 1;
    if (Begin + LeftSize + 1 == End) { // leaf
      BuildTreeLeaf(P, CurrNode->Right, RootNode, BBoxRight, Begin + LeftSize, End, BitPath, Path);
    } else { // recurse on the right
      BuildTreeInner(P, CurrNode->Right, RootNode, BBoxRight, Begin + LeftSize, End, BitPath, Path);
    }
  }
}

template <node_type R> static u64
CountPath(kd_tree<R>* CurrNode, kd_tree<Root>* RootNode, const bbox BBox, i64 Begin, i64 End, int D) {
  REQUIRE(Begin < End); // this cannot be a leaf node
  CurrNode->Begin = Begin;
  CurrNode->End = End;
  CurrNode->Left = CurrNode->Right = nullptr;
  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P3) { return P3.Pos[D] < Middle; };
  auto Right = partition(RootNode->Particles->begin() + Begin, RootNode->Particles->begin() + End, Pred);
  i64 LeftSize = Right - (RootNode->Particles->begin() + Begin);
  u64 PathLeft = 0, PathRight = 0;
  if (LeftSize > 0) {
    CurrNode->Left = new kd_tree<Inner>();
    auto BBoxLeft = BBox;
    BBoxLeft.Max[D] = Middle;
    if (LeftSize == 1) { // leaf
      PathLeft = D;
    } else { // recurse on the left
      PathLeft = CountPath(CurrNode->Left, RootNode, BBoxLeft, Begin, Begin + LeftSize, (D + 1) % 3);
      PathLeft = PathLeft * 3 + D;
    }
  }
  if (Begin + LeftSize < End) {
    CurrNode->Right = new kd_tree<Inner>();
    auto BBoxRight = BBox;
    BBoxRight.Min[D] = Middle;
    if (Begin + LeftSize + 1 == End) { // leaf
      PathRight = D;
    } else { // recurse on the right
      PathRight = CountPath(CurrNode->Right, RootNode, BBoxRight, Begin + LeftSize, End, (D + 1) % 3);
      PathRight = PathRight * 3 + D;
    }
  }
  return MAX(PathLeft, PathRight);
}

static kd_tree<Root>
BuildTree(const params& P, vector<particle>* Particles) {
  kd_tree<Root> Tree;
  Tree.Particles = Particles;
  Tree.BBox = ComputeBoundingBox(*Particles);
  u64 Path = CountPath(&Tree, &Tree, Tree.BBox, 0, Particles->size(), 0);
  BuildTreeInner(P, &Tree, &Tree, Tree.BBox, 0, Particles->size(), 0, Path);
  FOR (size_t, I, 0, ParticlesLODs.size()) {
    WriteXYZ(PRINT("particles-%zd.xyz", I), RANGE(ParticlesLODs[I]));
  }
  return Tree;
}

static void
RandomLevels(vector<float3>* Points) {
  random_device Rd;
  mt19937 G(Rd());
  shuffle(Points->begin(), Points->end(), G);
  auto Size = Points->size();
  FOR (int, I, 0, 5) {
    WriteXYZ(PRINT("random-particles-%d.xyz", I), RANGE(*Points, Size / 2, Size));
    Size /= 2;
  }
}

  //  void build(string order, A...)(Vec3!T[] points, A a) {
  //    order_ = order;
  //    points_ = points;
  //    bbox_ = compute_bbox(points);
  //    build_helper!(order)(this, bbox_, 0, cast(int)points.length, 0, a); // first split
  //  }

  //  void build_no_leaf(string order, A...)(Vec3!T[] points) {
  //    order_ = order;
  //    points_ = points;
  //    bbox_ = compute_bbox(points);
  //    build_helper_no_leaf!(order)(this, bbox_, 0, cast(int)points.length, 0); // first split
  //  }

  //  void build_with_bbox(string order, A...)(Vec3!T[] points, BoundingBox!T bbox, A a) {
  //    order_ = order;
  //    points_ = points;
  //    bbox_ = bbox;
  //    build_helper!(order)(this, bbox_, 0, cast(int)points.length, 0, a); // first split
  //  }
  //}

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
  params P;
  auto Particles = ReadXYZ(Argv[1]);
  printf("number of particles = %zu", Particles.size());
  BuildTree(P, &Particles);
  //RandomLevels(P, &Particles);
}

