// multiresolution-tree.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// TODO: better memory allocation (to put the leaves on the same memory block)

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

//template <typename t>
//struct range {
//  t Begin, End;
//  INLINE i64 size() const { return End - Begin; }
//  INLINE t  begin() const { return Begin; }
//  INLINE t    end() const { return End  ; }
//};

#define RANGE(...) MACRO_OVERLOAD(RANGE, __VA_ARGS__)
#define RANGE_0()
#define RANGE_1(Container) begin(Container), end(Container)
#define RANGE_2(Begin, End) Begin, End
#define RANGE_3(Container, Begin, End) begin(Container) + Begin, begin(Container) + End

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
struct resolution_tree {
  resolution_tree<Inner>* Left   = nullptr;
  resolution_tree<Inner>* Right  = nullptr;
  i64 Begin = 0, End = 0;
  using bbox_t = conditional_t<R == Root, bbox, empty_struct>;
  [[no_unique_address]] bbox_t BBox = bbox_t();
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
ComputeBoundingBox(const vector<particle>& Particles) {
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

unordered_map<u64, vector<i64>> ParticlesLODs;
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

using particles = vector<unordered_map<u64, vector<vec3f>>>; // [level] -> [blocks of particles]
static void
Refine(particles* Particles) {

}

//static void
//AssignCodeLeaf(vector<particle>* Particles, bbox BBox, i64 Begin, i64 End, u64 Code, i8 Length, i8 D) {
//  REQUIRE(Begin + 1 == End);
//  particle& P = (*Particles)[Begin];
//  while (Length--) {
//    auto Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
//    bool Left = P.Pos[D] < Middle;
//    Code = (Code << 1) + Left;
//    if (Left) BBox.Max[D] = Middle; else BBox.Min[D] = Middle;
//    D = (D + 1) % 3;
//  }
//  P.Code = Code;
//  //u64 Level = Lsb(Code);
//  //u64 BlockId = BLOCK_ID(Level, Code, Params.BlockBits);
//  //ParticlesLODs[BlockId].reserve(128);
//  //ParticlesLODs[BlockId].push_back(Begin);
//}

//static void
//AssignCodeInner(vector<particle>* Particles, const bbox BBox, i64 Begin, i64 End, u64 Code, i8 Height, i8 D) {
//  REQUIRE(Begin < End); // this cannot be a leaf node
//  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
//  auto Pred = [D, Middle](const particle& P) { return P.Pos[D] < Middle; };
//  i64 Mid = partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
//  if (Begin < Mid) {
//    if (Begin + 1 == Mid) { // leaf
//      AssignCodeLeaf(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, Code << 1, Height - 1, (D + 1) % 3);
//    } else { // recurse on the left
//      AssignCodeInner(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, Code << 1, Height - 1, (D + 1) % 3);
//    }
//  }
//  if (Mid < End) {
//    if (Mid + 1 == End) { // leaf
//      AssignCodeLeaf(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, (Code << 1) + 1, Height - 1, (D + 1) % 3);
//    } else { // recurse on the right
//      AssignCodeInner(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, (Code << 1) + 1, Height - 1, (D + 1) % 3);
//    }
//  }
//}

static void
BuildTreeLeaf(resolution_tree<Inner>* Node, vector<particle>* Particles, const bbox& BBox, vec3i& Dims3, i64 Begin, const grid& Grid, i8 D, i8 NLevels) {
  Node->Begin = Begin;
  Node->End = Begin + 1;
  // TODO: continue the splitting until we reach the desired accuracy
}

static grid
SplitGrid(const grid& Grid, int D, bool RSplit, bool Left) {
  REQUIRE((Grid.Stride3[D] & 1) == 0);
  grid Out = Grid;
  if (RSplit) { // resolution split
    // TODO
  } else { // spatial split
    // TODO
  }
  return Out;
}

template <node_type R> static void
BuildTreeInner(resolution_tree<R>* Node, vector<particle>* Particles, const bbox& BBox, vec3i& Dims3, i64 Begin, i64 End, const grid& Grid, i8 D, i8 NLevels, bool RSplit) {
  REQUIRE((Grid.Stride3[D] & 1) == 0);
  Node->Begin = Begin;
  Node->End = End;
  i64 Mid = Begin;
  if (RSplit) {
    auto RPred = [D, &Dims3, &Grid, &BBox](const particle& P) { 
      int Bin = MIN(Dims3[D] - 1, int((P.Pos[D] - BBox.Min[D]) / (BBox.Max[D] - BBox.Min[D])));
      REQUIRE((Bin - Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin - Grid.From3[D]) / Grid.Stride3[D];
      return (Bin & 1) == 0; 
    };
    Mid = partition(RANGE(*Particles, Begin, End), RPred) - Particles->begin();
  } else {
    auto SPred = [D, &Dims3, &Grid, &BBox](const particle& P) { 
      int Bin = MIN(Dims3[D] - 1, int((P.Pos[D] - BBox.Min[D]) / (BBox.Max[D] - BBox.Min[D])));
      REQUIRE((Bin - Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin - Grid.From3[D]) / Grid.Stride3[D];
      return Bin * 2 < Grid.Dims3[D]; 
    };
    Mid = partition(RANGE(*Particles, Begin, End), SPred) - Particles->begin();
  }
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // leaf
      Node->Left = new resolution_tree<Inner>();
      BuildTreeLeaf(Node->Left, Particles, BBox, Dims3, Begin, SplitGrid(Grid, D, RSplit, true), (D + 1) % 3, NLevels - 1);
    } else { // recurse on the left
      Node->Left = new resolution_tree<Inner>();
      BuildTreeInner(Node->Left, Particles, BBox, Dims3, Begin, Mid, SplitGrid(Grid, D, RSplit, true), (D + 1) % 3, NLevels - 1, NLevels > 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // leaf
      Node->Right = new resolution_tree<Inner>();
      BuildTreeLeaf(Node->Right, Particles, BBox, Dims3, Mid, SplitGrid(Grid, D, RSplit, false), (D + 1) % 3, NLevels - 1);
    } else { // recurse on the right
      Node->Right = new resolution_tree<Inner>();
      BuildTreeInner(Node->Right, Particles, BBox, Dims3, Mid, End, SplitGrid(Grid, D, RSplit, false), (D + 1) % 3, NLevels - 1, false);
    }
  }
}

/* Return the tree height and the dimensions of the underlying grid (in terms of power of two) */
static vec3i
ComputeGrid(vector<particle>* Particles, const bbox& BBox, i64 Begin, i64 End, i8 D) {
  REQUIRE(Begin < End); // this cannot be a leaf node
  float Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P) { return P.Pos[D] < Middle; };
  i64 Mid = partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left(0, 0, 0), LogDims3Right(0, 0, 0);
  if (Begin + 1 < Mid) {
    LogDims3Left = ComputeGrid(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, (D + 1) % 3);
    ++LogDims3Left[D];
  }
  if (Mid + 1 < End) {
    LogDims3Right = ComputeGrid(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, (D + 1) % 3);
    ++LogDims3Right[D];
  }
  return sum(LogDims3Left) > sum(LogDims3Right) ? LogDims3Left : LogDims3Right;
}

static void
RandomLevels(vector<vec3f>* Points) {
  random_device Rd;
  mt19937 G(Rd());
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
  params Params;
  auto Particles = ReadXYZ(Argv[1]);
  printf("number of particles = %zu\n", Particles.size());
  auto BBox = ComputeBoundingBox(Particles);
  auto LogDims3 = ComputeGrid(&Particles, BBox, 0, Particles.size(), 0);
  vec3i Dims3(1 << LogDims3.x, 1 << LogDims3.y, 1 << LogDims3.z);
  grid Grid{vec3i(0, 0, 0), vec3i(1, 1, 1), Dims3};
  int NLevels = 3; // actually number of resolution splits
  resolution_tree<Root> Tree;
  BuildTreeInner(&Tree, &Particles, BBox, Dims3, 0, Particles.size(), Grid, 0, NLevels, NLevels > 0);
  printf("log dims 3 = %d %d %d\n", LogDims3.x, LogDims3.y, LogDims3.z);
  //RandomLevels(P, &Particles);
}

