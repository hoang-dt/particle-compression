// multiresolution-tree.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// TODO: better memory allocation (to put the leaves on the same memory block)
// TODO: memory deallocation?
//#define ZFP
//#define ZFP_ONE_PARTICLE_PER_CELL
#define NO_ZFP
#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#define SEXPR_IMPLEMENTATION
#include "common.h"
#include "zfp.h"
#include "rans64.h"
#include "platform.h"
#include <algorithm>
struct stream {
  bitstream Stream;
  arithmetic_coder<> Coder;
  u32 StreamSize = 0;
  u32 CoderSize = 0;
};
static stream GlobalStream;
static std::vector<stream> Streams; // one for each block

static std::vector<particle_int> OutputParticles;
struct particle_id_and_bbox {
  i64 Id = 0;
  bbox_int BBox;
};
struct block_info {
  i32 NParticles = 0;
  i32 NParticlesDecoded = 0;
  std::vector<bbox_int> BBoxes;
  std::vector<u32> ParticleCount;
  std::vector<particle_id_and_bbox> BBoxesAndIds;
  std::vector<u64> PId; // particle id
  int BitCount = 0;
  bool AtRefinement = false;
};
static std::vector<block_info> OutputBlocks;

static bbox
ComputeBoundingBox(const std::vector<particle>& Particles) {
  REQUIRE(!Particles.empty());
  bbox BBox;
  BBox.Min = BBox.Max = Particles[0].Pos;
  FOR_EACH (P3, Particles) {
    BBox.Min = min(BBox.Min, P3->Pos);
    BBox.Max = max(BBox.Max, P3->Pos);
  }
  vec3f D3 = BBox.Max - BBox.Min;
  vec3f Adjustment3 = D3 / 16384; // enlarge the bounding box a little bit to avoid numerical issue at the boundary
  BBox.Min -= Adjustment3;
  BBox.Max += Adjustment3;
  return BBox;
}

static bbox_int
ComputeBoundingBox(const std::vector<particle_int>& Particles) {
  REQUIRE(!Particles.empty());
  bbox_int BBox;
  BBox.Min = BBox.Max = Particles[0].Pos;
  FOR_EACH(P3, Particles) {
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

/* Bit set stuffs */
using word = u64;
enum { WORD_SIZE = sizeof(word) * 8, WORD_LOG = 6, WORD_MASK = 0x3F };
struct bitset { word* Words; i64 NBits; };
#define NBITS_TO_NWORDS(NBits) (((NBits) + WORD_MASK) >> WORD_LOG)
#define NBYTES(BitSet) (sizeof(*(BitSet)->Words) * NBITS_TO_NWORDS((BitSet)->NBits))

INLINE static i64 BIndex(i64 B)  { return B >> WORD_LOG; }
INLINE static i64 BOffset(i64 B) { return B & WORD_MASK; }

//INLINE static void SetBit(bitset* BitSet, i64 B) { BitSet->Words[BIndex(B)] |= 1 << (BOffset(B)); }
//INLINE static void ClearBit(bitset* BitSet, i64 B) { BitSet->Words[BIndex(B)] &= ~(1 << (BOffset(B))); }
//INLINE static bool GetBit(const bitset* BitSet, i64 B) { return BitSet->Words[BIndex(B)] & (1 << (BOffset(B))); }
//INLINE static void ClearAll(bitset* BitSet) { memset(BitSet->Words, 0, NBYTES(BitSet)); }
//INLINE static void SetAll(bitset* BitSet) { memset(BitSet->Words, 0xFF, NBYTES(BitSet)); }

//static bitset*
//BitSetAlloc(i64 NBits) {
//  bitset* BitSet = (bitset*)malloc(sizeof(*BitSet));
//  BitSet->NBits = NBits;
//  BitSet->Words = (word*)malloc(NBYTES(BitSet));
//  ClearAll(BitSet);
//  return BitSet;
//}

//static void
//BitSetFree(bitset* BitSet) {
//  free(BitSet->Words);
//  free(BitSet);
//}

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
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "dimensions")) {
          REQUIRE(Expr->type == SE_INT);
          Params.NDims = Expr->i;
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "dims-string")) {
          strncpy(Params.DimsStr, (cstr)Buf.Data + Expr->s.start, Expr->s.len);
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
          Params.LogDims3.x = LOG2_FLOOR(Params.Dims3.x);
          Params.LogDims3.y = LOG2_FLOOR(Params.Dims3.y);
          Params.LogDims3.z = LOG2_FLOOR(Params.Dims3.z);
          Params.BaseHeight = Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z;
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "w3")) {
          REQUIRE(Expr->type == SE_INT);
          Params.W3.x = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          REQUIRE(Expr->type == SE_INT);
          Params.W3.y = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          REQUIRE(Expr->type == SE_INT);
          Params.W3.z = Expr->i;
          printf("W3 = %d %d %d\n", EXPvec3(Params.W3));
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "accuracy")) {
          REQUIRE(Expr->type == SE_FLOAT);
          Params.Accuracy = Expr->f;
          printf("Accuracy = %.8g\n", Params.Accuracy);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "refinement")) {
          REQUIRE(Expr->type == SE_INT);
          Params.RefinementMode = (refinement_mode)Expr->i;
          printf("Refinement = %d\n", Params.RefinementMode);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "height")) {
          REQUIRE(Expr->type == SE_INT);
          Params.MaxHeight = Expr->i;
          printf("Max height = %d\n", Params.MaxHeight);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "bounding-box")) {
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Min.x = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Min.y = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Min.z = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Max.x = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Max.y = Expr->i;
          REQUIRE(Expr->next);
          Expr = Expr->next;
          assert(Expr->type == SE_FLOAT || Expr->type == SE_INT);
          Params.BBoxInt.Max.z = Expr->i;
          //printf("bounding-box %f %f %f %f %f %f\n", EXPvec3(Params.BBox.Min), EXPvec3(Params.BBox.Max));
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "particles")) {
          REQUIRE(Expr->type == SE_INT);
          Params.NParticles = Expr->i;
          printf("particles = %lld\n", Params.NParticles);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "resolutions")) {
          REQUIRE(Expr->type == SE_INT);
          Params.NLevels = Expr->i;
          printf("resolutions = %d\n", Params.NLevels);
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "start-depth")) {
          REQUIRE(Expr->type == SE_INT);
          Params.StartResolutionSplit = Expr->i;
          printf("start-depth = %d\n", Params.StartResolutionSplit);
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

INLINE static vec3i
GenerateOneParticle(const bbox_int& BBox) {
  vec3i P3 = (BBox.Max + BBox.Min) / 2;
  return P3;
}

INLINE static void
GenerateOneParticle(const bbox& BBox) {
  //f32 Rx = f32(rand()) / f32(RAND_MAX);
  //f32 Ry = f32(rand()) / f32(RAND_MAX);
  //f32 Rz = f32(rand()) / f32(RAND_MAX);
  //vec3f P3 = BBox.Min + (BBox.Max - BBox.Min) * vec3f(Rx, Ry, Rz);
  vec3f P3 = BBox.Min + (BBox.Max - BBox.Min) * 0.5f;
  Particles.push_back(particle{.Pos = P3});
}

static void
GenerateParticlesPerNode(i64 N, const grid_int& Grid, std::vector<particle_int>* Output) {
  if (N == 0) return;
  assert(Grid.Dims3.x>=1 && Grid.Dims3.y>=1 && Grid.Dims3.z>=1);
  static std::vector<vec3i> GridPoints; // stores the grid points that contain the (to be generated) particles
  vec3i Dims3 = Grid.Dims3;
  
  i64 NElems = N;
  i64 I = 0;
  FOR(i32, I, 0, N) {
    i32 X = rand() % Dims3.x;
    i32 Y = rand() % Dims3.y;
    i32 Z = rand() % Dims3.z;
    vec3i P3 = Grid.From3 + Grid.Stride3*vec3i(X,Y,Z);
    bbox_int BBox{
      .Min = Params.BBoxInt.Min + P3*Params.W3,
      .Max = Params.BBoxInt.Min + (P3+1)*Params.W3 // TODO -1
    };
    Output->push_back(particle_int{.Pos=(BBox.Min+BBox.Max)/2});
    // TODO: here we don't really care if we have duplicated points
  }
}

struct Range {
  u64 From, To;
};

static vec3i
ComputeGrid(
  std::vector<particle_int>* Particles, const bbox_int& BBox, 
  i64 Begin, i64 End, i8 Depth, str DimsStr)
{
  REQUIRE(Begin < End); // this cannot be a leaf node
  REQUIRE(Params.StartResolutionSplit % Params.NDims == 0);
  i8 D = 0;
  vec3i BBoxExt3 = BBox.Max - BBox.Min;
  D = Depth % Params.NDims;
  if (BBoxExt3[D] <= 1) {
    D = (D+1) % Params.NDims;
    if (BBoxExt3[D] <= 1) {
      D = (D+1) % Params.NDims;
      REQUIRE(BBoxExt3[D] > 1);
    }
  }
  //if (Depth < Params.StartResolutionSplit) { // cycle through x/y/z when it's not resolution split yet
  //  D = Depth % Params.NDims;
  //} else { // take the largest dimension
  //  vec3i BBoxExt3 = BBox.Max - BBox.Min + 1;
  //  if (BBoxExt3.x>=BBoxExt3.y && BBoxExt3.x>=BBoxExt3.z)
  //    D = 0;
  //  else
  //  if (BBoxExt3.y>=BBoxExt3.z && BBoxExt3.y>=BBoxExt3.x)
  //    D = 1;
  //  else if (BBoxExt3.z>=BBoxExt3.y && BBoxExt3.z>=BBoxExt3.x)
  //    D = 2;
  //}
  DimsStr[Depth] = 'x' + D;
  //assert((BBoxExt3[D]&1) == 0);
  i32 Middle = (BBox.Min[D]+BBox.Max[D]) >> 1;
  auto Pred = [D, Middle](const particle_int& P) { return P.Pos[D] < Middle; };
  i64 Mid = std::partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left  = MCOPY(vec3i(0), [D]=1);
  vec3i LogDims3Right = MCOPY(vec3i(0), [D]=1);
  if (Begin+1 < Mid) {
    LogDims3Left = ComputeGrid(Particles, MCOPY(BBox, .Max[D]=Middle), Begin, Mid, Depth+1, DimsStr);
    ++LogDims3Left[D];
  }
  if (Mid+1 < End) {
    LogDims3Right = ComputeGrid(Particles, MCOPY(BBox, .Min[D]=Middle), Mid, End, Depth+1, DimsStr);
    ++LogDims3Right[D];
  }
  return max(LogDims3Left, LogDims3Right);
}

static void
RandomLevels(std::vector<particle>* Points) {
  std::random_device Rd;
  std::mt19937 G(Rd());
  shuffle(Points->begin(), Points->end(), G);
  auto Size = Points->size();
  FOR (int, I, 0, 10) {
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

#include "kdtree.h"


static bool
CheckSame( 
  std::vector<particle_int>& Particles1, std::vector<particle_int>& Particles2)
{
  if (Particles2.size() != Particles1.size())
    return false;
  std::sort(Particles1.begin(), Particles1.end(), [](const auto& P1, const auto& P2) {
    return Pack3i64(P1.Pos) < Pack3i64(P2.Pos);
  });
  std::sort(Particles2.begin(), Particles2.end(), [](const auto& P1, const auto& P2) {
    return Pack3i64(P1.Pos) < Pack3i64(P2.Pos);
  });
  for (size_t I = 0; I < Particles1.size(); ++I) {
    vec3i Diff = Particles1[I].Pos - Particles2[I].Pos;
    if (Diff.x!=0 || Diff.y!=0 || Diff.z!=0)
      return false;
  }
  return true;
}

static f32
Error3( 
  const std::vector<particle_int>& Particles1, const std::vector<particle_int>& Particles2, const vec3i& Dims3)
{
  printf("error3\n");
  std::vector<point<int, 3>> Pos(Particles1.size());
  FOR(i64, I, 0, Particles1.size()) {
    Pos[I].coords_[0] = Particles1[I].Pos[0];
    Pos[I].coords_[1] = Particles1[I].Pos[1];
    Pos[I].coords_[2] = Particles1[I].Pos[2];
  }
  kdtree<int, 3> Tree(Pos.begin(), Pos.end());
  f64 Err = 0;
  FOR_EACH(P, Particles2) {
    point<int, 3> Point;
    Point.coords_[0] = P->Pos.x;
    Point.coords_[1] = P->Pos.y;
    Point.coords_[2] = P->Pos.z;
    auto Nearest = Tree.nearest(Point);
    vec3i Q(Nearest.coords_[0], Nearest.coords_[1], Nearest.coords_[2]);
    vec3i Diff = Q - P->Pos;
    Err += Diff.x * Diff.x + Diff.y * Diff.y + Diff.z * Diff.z;
  }
  auto BBox = ComputeBoundingBox(Particles2);
  auto D3 = BBox.Max - BBox.Min;
  auto M = MAX(D3.x, D3.y);
  M = MAX(M, D3.z);
  int NDims = Dims3.z == 1 ? 2 : 3;
  Err = std::sqrt(Err / (NDims * Particles2.size()));
  double Psnr = 20 * log10(M/Err);
  printf("%f\n", Psnr);
  return f32(Err);
}

static f32
Error3( 
  const std::vector<particle>& Particles1, const std::vector<particle>& Particles2, const vec3i& Dims3)
{
  std::vector<point<float, 3>> Pos(Particles1.size());
  FOR(i64, I, 0, Particles1.size()) {
    Pos[I].coords_[0] = Particles1[I].Pos[0];
    Pos[I].coords_[1] = Particles1[I].Pos[1];
    Pos[I].coords_[2] = Particles1[I].Pos[2];
  }
  kdtree<float, 3> Tree(Pos.begin(), Pos.end());
  float Err = 0;
  FOR_EACH(P, Particles2) {
    point<float, 3> Point;
    Point.coords_[0] = P->Pos.x;
    Point.coords_[1] = P->Pos.y;
    Point.coords_[2] = P->Pos.z;
    auto Nearest = Tree.nearest(Point);
    vec3f Q(Nearest.coords_[0], Nearest.coords_[1], Nearest.coords_[2]);
    vec3f Diff = Q - P->Pos;
    Err += Diff.x * Diff.x + Diff.y * Diff.y + Diff.z * Diff.z;
  }
  int NDims = Dims3.z == 1 ? 2 : 3;
  Err = std::sqrt(Err / (NDims * Particles2.size()));
  return Err;
}

/* NOTE: Particles2 should be the reference */
static f32
Error(
  const bbox& BBox, 
  const std::vector<particle>& Particles1, const std::vector<particle>& Particles2, const vec3i& Dims3)
{
  //bbox BBox = ComputeBoundingBox(Particles1);
  vec3f W3 = (BBox.Max - BBox.Min) / vec3f(Dims3);
  std::vector<vec3f> Grid(Dims3.x * Dims3.y * Dims3.z, vec3f(NAN));
  FOR_EACH(P, Particles1) {
    vec3i Coord{
      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
    i32 Idx = Coord.z * (Dims3.x * Dims3.y) + Coord.y * (Dims3.x) + Coord.x;
    assert(Grid[Idx].x != Grid[Idx].x);
    Grid[Idx] = P->Pos;
  }
  float Err = 0;
  FOR_EACH(P, Particles2) {
    vec3i Coord{
      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
    i32 MaxD = MAX(Dims3.z, MAX(Dims3.x, Dims3.y)) / 2;
    for (i32 D = 0; D < MaxD; ++D) {
      f32 MinDiff = INFINITY;
      bool Found = false;
      for (i32 Dz = -D; Dz <= D; ++Dz) {
        i32 Z = Coord.z + Dz;
        if (Z < 0 || Z >= Dims3.z) continue;
        bool EdgeZ = (Dz == -D) || (Dz == D) || (Z == 0) || (Z == Dims3.z - 1);
        for (i32 Dy = -D; Dy <= D; ++Dy) {
          i32 Y = Dy + Coord.y;
          if (Y < 0 || Y >= Dims3.y) continue;
          bool EdgeY = (Dy == -D) || (Dy == D) || (Y == 0) || (Y == Dims3.y - 1);
          bool Edge = EdgeZ || EdgeY;
          for (i32 Dx = -D; Dx <= D; ++Dx) {
            i32 X = Dx + Coord.x;
            if (X < 0 || X >= Dims3.x) continue;
            bool EdgeX = (Dx == -D) || (Dx == D) || (X == 0) || (X == Dims3.x - 1);
            if (!(Edge || EdgeX)) continue;
            i32 Idx = Z * (Dims3.x * Dims3.y) + Y * (Dims3.x) + X;
            if (Grid[Idx].x == Grid[Idx].x) { // not NAN
              vec3f Diff = Grid[Idx] - P->Pos;
              MinDiff = MIN(MinDiff, Diff.x * Diff.x + Diff.y * Diff.y + Diff.z * Diff.z);
              Found = true;
            }
          }
        }
      }
      if (Found) {
        Err += MinDiff;
        break;
      } else {
        //assert(false);
      }
    }
  }
  int NDims = Dims3.z == 1 ? 2 : 3;
  Err = std::sqrt(Err / (NDims * Particles2.size()));
  return Err;
}

void WriteBlockNew(bitstream* Bs, u64 BlockIdx);
void FlushBlocksToFilesNew();
void BuildTreeNew(q_item_new Q, float Accuracy);
void EncodeRootNew(i64 N);

f64 RMSE = 0;

static cdf_table CdfTable;
static std::vector<cdf_table> BinomialTables;
static std::vector<std::vector<std::vector<f64>>> BinomialTablesF64;
static arithmetic_coder<> Coder;
//static arithmetic_coder<> Coder2;

/* generate random particles in a grid of 512^3, with a given density */
static std::vector<particle_int>
GenerateRandomParticles(i32 Prob) { // Prob is between 0 and 100
  //REQUIRE(Prob >= 0);
  //REQUIRE(Prob <= 100);
  printf("prob = %d\n", Prob);
  std::vector<particle_int> Result;
  Result.reserve(128 * 128 * 128);
  FOR(i32, I, 0, 128) {
    FOR(i32, J, 0, 128) {
      FOR(i32, K, 0, 128) {
        if ((rand() % 100) < Prob) {
          Result.push_back(particle_int{.Pos = {I, J, K}});
        } else {
          int Stop = 0;
        }
      }
    }
  }
  return Result;
}

i64 Counts[33] = {};
i64 ZeroCount = 0;
i64 TotalCount = 0;
#define SPLIT SpatialSplit
#define THRESHOLD 0 // where we switch from spatial split to resolution split
#define THRESHOLD2 0
f64 SeparationCodeLength = 0;
i64 RefinementCodeLength = 0;
f64 Ratio = 0;
i64 RatioCount = 0;
i64 NodesWithMoreEmptyCellsCount = 0;
i64 NodesWithMoreParticlesCount = 0;

static constexpr int MaxDepth = 32;
static constexpr int Threshold = 1 << 10;
static u32 ProbCount[MaxDepth][Threshold + 1][Threshold + 1] = {};
static u32 SingleProbCount[Threshold + 1] = {};
static u32 ProbTotal = 0;

inline void
CreateProbTable() {
  FOR(int, L, 0, MaxDepth)
    FOR(int, N, 0, Threshold + 1)
      FOR(int, K, 1, N + 1)
        ProbCount[L][N][K] += ProbCount[L][N][K-1];
  //FOR(int, N, 1, Threshold + 1) {
  //  SingleProbCount[N] += SingleProbCount[N - 1];
  //  ProbTotal = SingleProbCount[N];
  //}
}

static f64 CodeLengthPerLevel[32] = {};

static tree* TreePtr = nullptr, *TreePtrBackup = nullptr;
static tree* PrevFramePtr = nullptr, *PrevFramePtrBackup = nullptr;

/* Node and RefNode should be at the same relative position on the trees 
* FirstBranch = the first split
* LastBranch  = the last split so far */
enum class branch{ Left, Right };
static tree*
BuildPredTreeRecursive(branch LastBranch, tree* RefNode, tree* Node, i8 Depth, i8 LastD) {
  // traverse the three trees in the same way
  //printf("Depth = %d\n", int(Depth));
  if (!RefNode && !Node) {
    return nullptr;
  }
  if (Depth == Params.MaxDepth) { // leaf node
    //REQUIRE(!(RefNode && Node));
    if (RefNode || Node) { // only one of the two should be non null
      tree* PredNode = new (TreePtr++) tree;
      PredNode->Count = RefNode ? RefNode->Count : Node->Count;
      return PredNode;
    }
  } else { // inner node, simply sum up the left and right branches
    tree* LeftRef = nullptr, *LeftNode = nullptr, *RightRef = nullptr, *RightNode = nullptr;
    if (RefNode) {
      LeftRef  = RefNode->Left;
      RightRef = RefNode->Right;
    }
    if (Node) {
      LeftNode  = Node->Left;
      RightNode = Node->Right;
    }
    tree* LeftPredNode = nullptr, *RightPredNode = nullptr;
    if (Depth == LastD) { // right before the last split in D happen
    //if (false) { // right before the last split in D happen
      if (LastBranch == branch::Left) { // we go left on both PredNode and Node
        LeftPredNode  = BuildPredTreeRecursive(branch::Left , LeftRef, nullptr, Depth + 1, LastD);
        RightPredNode = BuildPredTreeRecursive(branch::Right, nullptr, LeftNode, Depth + 1, LastD);
      } else { // LastBranch == branch::Right, go right on both PredNode and Node
        LeftPredNode  = BuildPredTreeRecursive(branch::Left , RightRef, nullptr, Depth + 1, LastD);
        RightPredNode = BuildPredTreeRecursive(branch::Right, nullptr, RightNode, Depth + 1, LastD);
      }
    } else if (Params.DimsStr[Depth] == Params.DimsStr[LastD]) { // not the last split in D, just go the same route (delayed by one)
      if (LastBranch == branch::Left) { // we go left on both PredNode and Node
        LeftPredNode  = BuildPredTreeRecursive(branch::Left , LeftRef, LeftNode, Depth + 1, LastD);
        RightPredNode = BuildPredTreeRecursive(branch::Right, LeftRef, LeftNode, Depth + 1, LastD);
      } else { // LastBranch == branch::Right, go right on both PredNode and Node
        LeftPredNode  = BuildPredTreeRecursive(branch::Left , RightRef, RightNode, Depth + 1, LastD);
        RightPredNode = BuildPredTreeRecursive(branch::Right, RightRef, RightNode, Depth + 1, LastD);
      }
    } else {
      LeftPredNode  = BuildPredTreeRecursive(LastBranch, LeftRef , LeftNode , Depth + 1, LastD);
      RightPredNode = BuildPredTreeRecursive(LastBranch, RightRef, RightNode, Depth + 1, LastD);
    }
    /* combine the two branches */
    if (!LeftPredNode && !RightPredNode) {
      return nullptr;
    }
    tree* PredNode  = new (TreePtr++) tree;
    PredNode->Left  = LeftPredNode;
    PredNode->Right = RightPredNode;
    if (LeftPredNode) PredNode->Count = LeftPredNode->Count; else PredNode->Count = 0;
    if (RightPredNode) PredNode->Count += RightPredNode->Count;
    return PredNode;
  }
  return nullptr;
}

// TODO: dealloc the pred tree
// TODO: construct DimsStr
// NOTE: Depth is of the parent node of RefNode and Node
static tree*
BuildPredTree(tree* RefNode, tree* Node, i8 Depth, i8 D) {
  tree* Root = new (TreePtr++) tree;
  i8 LastD = Params.MaxDepth;
  while ((LastD >= 0) && (Params.DimsStr[LastD] != 'x'+D)) --LastD;
  //REQUIRE(LastD > Depth); // TODO: what if LastD == Depth?
  Root->Left  = BuildPredTreeRecursive(branch::Left , RefNode, Node, Depth + 1, LastD);
  Root->Right = BuildPredTreeRecursive(branch::Right, RefNode, Node, Depth + 1, LastD);
  Root->Count = (Root->Left?Root->Left->Count:0) + (Root->Right?Root->Right->Count:0);
  return Root;
}

static void
DumpTree(const tree* Node, bool FirstTime = false) {
  static FILE* Fp = fopen("tree.dat", "wb");
  if (Node) {
    //printf("%lld\n", Node->Count);
    fwrite(&Node->Count, sizeof(Node->Count), 1, Fp);
    if (Node->Left || Node->Right) {
      DumpTree(Node->Left, false);
      DumpTree(Node->Right, false);
    }
  } else {
    //printf("0\n");
  }
  if (FirstTime) {
    if (Fp) fclose(Fp);
  }
}

struct debug_prob {
  u32 P = 0;
  u32 N = 0;
  u32 L = 0;
};

static i64
CountCodeLength(const std::vector<i32> ResidualTree) {
  f64 CodeLength = 0;
  for (int I = 1; I < ResidualTree.size(); I += 2) {
    int P = I / 2;
    CodeLength += log2(ResidualTree[P]+1);
  }
  return i64((CodeLength + 7) / 8);
}

static vec2i 
IndexRange(int level, int nlevels, int nleaves) {
  int begin = Pow(2, level) - 1;
  int end = level+1<nlevels ? begin*2+1 : begin+nleaves;
  return vec2i{begin, end};
}

static std::vector<debug_prob> DebugProbs; 
static i64 BinomialCodeSize = 0;
static f64 RangeCodeSize    = 0;
static i64 UniformCodeSize1 = 0;
static i64 UniformCodeSize2 = 0;
static i64 PredictedNodeCount = 0;
static i64 NonPredictedNodeCount = 0;
static i64 NonPredictedCodeSize = 0;
static f64 ResidualCodeLengthNormal = 0;
static f64 ResidualCodeLengthGamma = 0;
static u32* RansPtr = nullptr;
//#define RESOLUTION_ALWAYS 1
//#define RESOLUTION_PREDICT 1
//#define BINOMIAL 1
//#define FORCE_BINOMIAL 1
//#define PREDICTION  1
//#define TIME_PREDICT 1
#define NORMAL 1
//#define SOTA 1
//#define LIGHT_PREDICT 1
static std::vector<i32> Residuals;
static std::vector<std::vector<particle_int>> ParticleLevels;

static constexpr u32 ContextMax = 32;
static u32 ContextTSResolution[ContextMax][ContextMax] = {};
// [T][MM][KK][S]
//struct one_context_type {
//  u32 Array[ContextMax] = {};
//  u32& operator[](int I) { return Array[I]; }
//  const u32* data() const { return Array; }
//};
using one_context_type = std::array<u32, ContextMax+2>;
//using one_context_type = std::vector<u32>;
using context_elem_type = std::unordered_map<u32, one_context_type>;
//using context_elem_type_3 = std::array<std::array<std::array<one_context_type, ContextMax+2>, ContextMax+2>, ContextMax+2>;
//using context_elem_type_2 = std::array<std::array<one_context_type, ContextMax+2>, ContextMax+2>;
//using context_elem_type_1 = std::array<one_context_type, ContextMax+2>;
//using context_type_3 = std::vector<context_elem_type_3>; // one context for each resolution level
//using context_type_2 = std::vector<context_elem_type_2>; // one context for each resolution level
//using context_type_1 = std::vector<context_elem_type_1>; // one context for each resolution level
using context_type = std::vector<context_elem_type>; // one context for each resolution level
static context_type ContextS;
static context_type ContextR;
static context_type ContextTS;
static context_type ContextTSR;
//static u32 ContextR[ContextMax][ContextMax][ContextMax] = {};

static std::vector<bool> PredBuf; // prediction grid // TODO: replace with a more compact array
static std::vector<i8> CountGrid; // count grid should be half of PredGrid
static grid_int PredGrid;
std::vector<vec2i> SRList;

static i64 BlockCount = 0;
/* At certain depth, we split the node using the Resolution split into a number of levels, then use the
low-resolution nodes to predict the values for finer-resolution nodes */
static tree*
DecodeTreeIntPredict(
  const tree* PredNode, std::vector<particle_int>& Particles, i64 Begin, i64 End, i8 T, const grid_int& Grid, 
  split_type Split, i8 ResLvl, i8 Depth) 
{
  assert(ResLvl < Params.NLevels);
  assert(Depth <= Params.MaxDepth);
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  i8 D = Params.DimsStr[Depth] - 'x';

  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  assert(CellCountLeft+CellCountRight == CellCount);

  /* decode to find Mid */
#if defined(BINOMIAL)  || defined(FORCE_BINOMIAL)
  i64 N = End - Begin;
  i64 P = DecodeCenteredMinimal(u32(N+1), &BlockStream);
  i64 Mid = P + Begin;
  i8 S = 0, R = 0;
#elif defined(SOTA)
  i64 N = End - Begin;
  i64 P = DecodeCenteredMinimal(u32(N+1), &BlockStream);
  i64 Mid = P + Begin;
  i8 S = 0, R = 0;
#elif defined(NORMAL)
  i64 N = End - Begin;
  i64 Mid = Begin;
  i64 P = 0;
  bool Flip = false;
  if (CellCount-N < N) {
    Flip = true;
    N = CellCount - N;
  }
  //N = MIN(N, CellCountRight);
  P = DecodeCenteredMinimal(u32(N+1), &GlobalStream.Stream);
  if (Flip) {
    P = CellCountLeft - P;
  }
  Mid = P + Begin;
  i8 S = 0, R = 0;
#elif defined(LIGHT_PREDICT)
  i64 Mid = Begin;
  bool FullGrid = (T>0) && (1<<(T-1))==CellCount;
  bool EncodeEmptyCells = false;
  u32 CIdx = ResLvl*Params.NLevels + Depth;    
  i8 S = 0, R = 0;
  if (!FullGrid && T>0) { // no prediction, try 1-context
    ContextTS[CIdx][T][0] = 1;
    S = DecodeWithContext(T, ContextTS[CIdx][T].data(), &Coder);
    S = (S==0) ? DecodeCenteredMinimal(T+1, &BlockStream) : S-1;
    ++ContextTS[CIdx][T][S+1];
  } else if (FullGrid) {
    S = T - 1;
  } else { // if S == 0
    S = 0;
    //++ContextTS[CIdx][T][S+1];
  }

  if (FullGrid) {
    R = T - 1;
  } else if (T==1 && S==1) {
    R = 0;
  } else if (S == 0) {
    R = T;
  } else {
    u32 CR = T*(ContextMax+2) + S;
    ContextTSR[CIdx][CR][0] = 1;
    R = DecodeWithContext(T, ContextTSR[CIdx][CR].data(), &Coder);
    R = (R==0) ? DecodeCenteredMinimal(T+1, &BlockStream) : R-1;
    ++ContextTSR[CIdx][CR][R+1];
  }
#elif defined(PREDICTION) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  //static int SRCounter = 0;
  i64 Mid = Begin;
  bool FullGrid = (T>0) && (1<<(T-1))==CellCount;
  bool EncodeEmptyCells = false;
  u32 CIdx = ResLvl*Params.NLevels + Depth;    
  i8 S = 0, R = 0;
  //if (!FullGrid && T>0 && PredNode) { // predict P
  //  i64 M = PredNode->Count;
  //  i64 K = PredNode->Left?PredNode->Left->Count : M - PredNode->Right->Count;
  //  if (EncodeEmptyCells)  { K= CellCountLeft - K; M = CellCount - M; }
  //  i8 MM = Msb(u64(M)) + 1;
  //  i8 KK = Msb(u64(K)) + 1;
  //  u32 C = T*(ContextMax+2)*(ContextMax+2) + MM*(ContextMax+2) + KK;
  //  ContextS[CIdx][C][0] = 1;
  //  S = DecodeWithContext(T, ContextS[CIdx][C].data(), &Coder);
  //  S = (S==0) ? DecodeCenteredMinimal(T+1, &BlockStream) : S-1;
  //  ++ContextS[CIdx][C][S+1];
  //  //++ContextTS[CIdx][T][S+1];
  //} else if (!FullGrid && T>0) { // no prediction, try 1-context
  //  ContextTS[CIdx][T][0] = 1;
  //  S = DecodeWithContext(T, ContextTS[CIdx][T].data(), &Coder);
  //  S = (S==0) ? DecodeCenteredMinimal(T+1, &BlockStream) : S-1;
  //  ++ContextTS[CIdx][T][S+1];
  //} else if (FullGrid) {
  //  S = T - 1;
  //} else { // if S == 0
  //  S = 0;
  //  //++ContextTS[CIdx][T][S+1];
  //}

  //if (FullGrid) {
  //  R = T - 1;
  //} else if (T==1 && S==1) {
  //  R = 0;
  //} else if (S == 0) {
  //  R = T;
  //} else {
  //  u32 CR = T*(ContextMax+2) + S;
  //  ContextR[CIdx][CR][0] = 1;
  //  R = DecodeWithContext(T, ContextR[CIdx][CR].data(), &Coder);
  //  R = (R==0) ? DecodeCenteredMinimal(T+1, &BlockStream) : R-1;
  //  ++ContextR[CIdx][CR][R+1];
  //}
#endif

  tree* SaveTreePtr = nullptr;
  if (Depth == Params.StartResolutionSplit) { // beginning of block
    SaveTreePtr = TreePtr;
    ++BlockCount;
    //REQUIRE(Split == ResolutionSplit);
    //FOR_EACH(Context, ContextS ) { Context->clear(); }
    //FOR_EACH(Context, ContextTS) { Context->clear(); }
    //FOR_EACH(Context, ContextR ) { Context->clear(); }
    static bool Done = false;
    if (!Done) {
      printf("grid dims is %d %d %d\n", Grid.Dims3.x, Grid.Dims3.y, Grid.Dims3.z);
      Done = true;
    }
  }
  //const vec2i& SR = SRList[SRCounter++];
  //assert(S == SR.x);
  //assert(R == SR.y);

  /* recurse */
  tree* Left = nullptr; 
#if defined(LIGHT_PREDICT) || defined(TIME_PREDICT)
  if (S == 1) {
#elif defined(PREDICTION) || defined(RESOLUTION_PREDICT)
  if (S==1 && CellCountLeft==1) {
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  if (Begin+1 == Mid) {
#endif
    assert(Depth+1 == Params.MaxDepth);
    Left = new (TreePtr++) tree;
    Left->Count = 1;
    ++NParticlesDecoded;
    bbox_int BBox;
    BBox.Min = Params.BBoxInt.Min + GridLeft.From3*Params.W3;
    BBox.Max = BBox.Min + GridLeft.Dims3*Params.W3 - 1;
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Read(&GlobalStream.Stream);
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M+1;
      }
    }
#if defined(PREDICTION) || defined(LIGHT_PREDICT) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  } else if (S >= 1) { //recurse
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  } else if (Begin < Mid) {
#endif
    assert(Depth+1 < Params.MaxDepth);
    split_type NextSplit = 
      ((Depth+1==Params.StartResolutionSplit) ||
       (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      Left = DecodeTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Left = DecodeTreeIntPredict(nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl+1, Depth+1);
  }

  /* recurse on the right */
  tree* Right = nullptr;
#if defined(LIGHT_PREDICT) || defined(TIME_PREDICT)
  if (R == 1) {
#elif defined(PREDICTION) || defined(RESOLUTION_PREDICT)
  if (R==1 && CellCountRight==1) {
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  if (Mid+1 == End) {
#endif
    assert(Depth+1 == Params.MaxDepth);
    Right = new (TreePtr++) tree;
    Right->Count = 1;
    ++NParticlesDecoded;
    bbox_int BBox;
    BBox.Min = Params.BBoxInt.Min + GridRight.From3*Params.W3; 
    BBox.Max = BBox.Min + GridRight.Dims3*Params.W3 - 1;
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Read(&GlobalStream.Stream);
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M+1;
      }
    }
#if defined(PREDICTION) || defined(LIGHT_PREDICT) ||defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  } else if (R >= 1) { //recurse
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL)  || defined(FORCE_BINOMIAL)
  } else if (Mid < End) {
#endif
    assert(Depth+1 < Params.MaxDepth);
    split_type NextSplit = (Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      Right = DecodeTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, R, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Right = DecodeTreeIntPredict(Left, Particles, Mid, End, R, GridRight, NextSplit, ResLvl+1, Depth+1);
  }

  /* construct the prediction tree */
  tree* Node = nullptr;
#if defined(PREDICTION)
  if (Split == ResolutionSplit) {
    Node = BuildPredTree(Left, Right, Depth, D);
    assert(Node && Node->Count>0);
  } else if (Split == SpatialSplit) {
    if (Depth > Params.StartResolutionSplit) {
      Node = new (TreePtr++) tree;
      Node->Left  = Left;
      Node->Right = Right;
      if (Left ) Node->Count = Left->Count; else Node->Count = 0;
      if (Right) Node->Count += Right->Count;
    }
  }
  if (Depth==Params.StartResolutionSplit && Node!=nullptr) {
    TreePtr = SaveTreePtr; // free memory
    *TreePtr = *Node;
    Node = TreePtr++;
  }
#endif

  return Node;
}

/* At certain depth, we split the node using the Resolution split into a number of levels, then use the
low-resolution nodes to predict the values for finer-resolution nodes */
static u32 NumNodeAllocated = 0;
static tree*
BuildTreeIntPredict(
  const tree* PredNode, std::vector<particle_int>& Particles, i64 Begin, i64 End, 
  i8 T, const grid_int& Grid, split_type Split, i8 ResLvl, i8 Depth)
{
  assert(ResLvl < Params.NLevels);
  assert(Depth <= Params.MaxDepth);
  i64 N = End - Begin; // total number of particles
  assert(Msb(u64(N))+1 == T);
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  //if (CellCount == N) return nullptr;
  i8 D = Params.DimsStr[Depth] - 'x';

  /* split in either resolution or precision */
  i64 Mid = Begin;
  i32 MM = Grid.From3[D]; // the beginning of the right child
  if (Split == ResolutionSplit) {
    auto RPred = [D, &Grid](const particle_int& P) {
      i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
      Bin = (Bin-Grid.From3[D]) / Grid.Stride3[D];
      return IS_EVEN(Bin);
    };
    Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else if (Split == SpatialSplit) {
    MM = Grid.From3[D] + (((Grid.Dims3[D]+1)>>1)-1) * Grid.Stride3[D];
    auto SPred = [MM, D, &Grid](const particle_int& P) {
      i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
      return Bin <= MM;
    };
    Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }

  /* encode */
  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  REQUIRE(CellCountLeft+CellCountRight == CellCount);
  i64 P = Mid - Begin;
  i8 S = Msb(u32(P)) + 1;
  i8 R = Msb(u32(N-P)) + 1;

#if defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  if (CellCount-N < N) {
    N = CellCount - N;
    P = CellCountLeft - P;
  }
  f64 Mean = f64(N) / 2; // mean
  f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
  //EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &BlockStream, &Coder);
  if (Split == ResolutionSplit) {
    f64 Mean = f64(N) / 2; // mean
    f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
    EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &BlockStream, &Coder);
  } else {
    EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
  }
#elif defined(SOTA)
  EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
#elif defined(NORMAL)
  if (CellCount-N < N) {
    N = CellCount - N;
    P = CellCountLeft - P;
  }
  //N = MIN(N, CellCountRight); // this only makes sense if the grid dimension is non power of two (so that the right can have fewer cells than the left)
  EncodeCenteredMinimal(u32(P), u32(N+1), &GlobalStream.Stream);
  //EncodeUniform(N, P, &Coder);
#elif defined(LIGHT_PREDICT)
  bool FullGrid = (T>0) && (1<<(T-1))==CellCount;
  bool EncodeEmptyCells = false;
  u32 CIdx = ResLvl*Params.NLevels + Depth;    
  //u32 CIdx = Depth;
  if (!FullGrid && T>0) { // no prediction, try 1-context
    if (ContextTS[CIdx][T][S+1] == 0) {
      ContextTS[CIdx][T][0] = 1;
      EncodeWithContext(T, 0, ContextTS[CIdx][T].data(), &Coder);
      EncodeCenteredMinimal(S, T+1, &BlockStream);  // TODO: try the binomial one
      //EncodeGeometric(T, S, &Coder);
      //EncodeUniform(T, S, &Coder);
    } else {
      ContextTS[CIdx][T][0] = 1;
      EncodeWithContext(T, S+1, ContextTS[CIdx][T].data(), &Coder);
    }
    ++ContextTS[CIdx][T][S+1];
  }

  if (T > 0) {
    u32 CR = T*(ContextMax+2) + S;
    if (FullGrid) {
      assert(R == T-1);
    } else if (T==1 && S==1) {
      assert(R == 0);
    } else if (S == 0) {
      assert(R == T);
    } else {
      if (ContextTSR[CIdx][CR][R+1] == 0) {
        ContextTSR[CIdx][CR][0] = 1;
        EncodeWithContext(T, 0, ContextTSR[CIdx][CR].data(), &Coder);
        EncodeCenteredMinimal(R, T+1, &BlockStream);
        //EncodeUniform(T, S, &Coder);
        //EncodeGeometric(T, R, &Coder);
      } else { // there is a context
        ContextTSR[CIdx][CR][0] = 1;
        EncodeWithContext(T, R+1, ContextTSR[CIdx][CR].data(), &Coder);
      }
      ++ContextTSR[CIdx][CR][R+1];
    }
  }
#elif defined(PREDICTION) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  //static int SRCounter = 0;
  bool FullGrid = (T>0) && (1<<(T-1))==CellCount;
  bool EncodeEmptyCells = false;
  //if ((1<<T) >= CellCount) { // more particles than empty cells
  //  EncodeEmptyCells= true;
  //  T = Msb(u64(CellCount));
  //  REQUIRE(T > 0);
  //  S = Msb(u64(CellCountLeft-P)) + 1;
  //  R = Msb(u64(CellCountRight+P-N)) + 1;
  //  REQUIRE(T >= S);
  //  REQUIRE(T >= R);
  //}
  u32 CIdx = ResLvl*Params.NLevels + Depth;    
  //u32 CIdx = Depth;
  //if (!FullGrid && N>2) {
  //  if (CellCount-N < N) {
  //    N = CellCount - N;
  //    P = CellCountLeft - P;
  //  }
  //  if (Split == ResolutionSplit) {
  //    f64 Mean = f64(N) / 2; // mean
  //    f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
  //    EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &BlockStream, &Coder);
  //  } else {
  //    EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
  //  }
  //} else
  if (!FullGrid && T>0 && PredNode && (PredNode->Left||PredNode->Right)) { // predict P
    i64 M = PredNode->Count;
    i64 K = PredNode->Left?PredNode->Left->Count : M - PredNode->Right->Count;
    i64 L = M - K;
    if (EncodeEmptyCells)  { K= CellCountLeft - K; M = CellCount - M; }
    i8 MM = Msb(u64(M)) + 1;
    i8 KK = Msb(u64(K)) + 1;
    i8 LL = Msb(u64(L)) + 1;
    {
      u32 C = T*(ContextMax+2)*(ContextMax+2)*(ContextMax+2) + MM*(ContextMax+2)*(ContextMax+2) + KK*(ContextMax+2) + LL;
      auto It = ContextS[CIdx].find(C);
      if (It == ContextS[CIdx].end()) {
        auto Pair = ContextS[CIdx].insert({C, one_context_type()});
        Pair.first->second[0] = 1;
        EncodeWithContext(T, 0, Pair.first->second.data(), &Coder);
        //EncodeCenteredMinimal(S, T+1, &BlockStream);
        EncodeUniform(T, S, &Coder);
        ++Pair.first->second[S+1];
      } else {
        It->second[0] = 1;
        if (It->second[S+1] == 0) {
          EncodeWithContext(T, 0, It->second.data(), &Coder);
          //EncodeCenteredMinimal(S, T+1, &BlockStream);
          EncodeUniform(T, S, &Coder);
        } else {
          EncodeWithContext(T, S+1, It->second.data(), &Coder);
        }
        ++It->second[S+1];
      }
    }
    { // encode R if necessary
       if (FullGrid) {
        assert(R == T-1);
      } else if (T==1 && S==1) {
        assert(R == 0);
      } else if (S == 0) {
        assert(R == T);
      } else {
        u32 C = T*(ContextMax+2)*(ContextMax+2)*(ContextMax+2)*(ContextMax+2) + MM*(ContextMax+2)*(ContextMax+2)*(ContextMax+2) + KK*(ContextMax+2)*(ContextMax+2) + (LL*ContextMax+2) + S;
        auto It = ContextR[CIdx].find(C);
        if (It == ContextR[CIdx].end()) {
          auto Pair = ContextR[CIdx].insert({C, one_context_type()});
          Pair.first->second[0] = 1;
          EncodeWithContext(T, 0, Pair.first->second.data(), &Coder);
          //EncodeCenteredMinimal(R, T+1, &BlockStream);
          EncodeUniform(T, R, &Coder);
          ++Pair.first->second[R+1];
        } else {
          It->second[0] = 1;
          if (It->second[R+1] == 0) {
            EncodeWithContext(T, 0, It->second.data(), &Coder);
            //EncodeCenteredMinimal(R, T+1, &BlockStream);
            EncodeUniform(T, R, &Coder);
          } else {
            EncodeWithContext(T, R+1, It->second.data(), &Coder);
          }
          ++It->second[R+1];
        }
      }
    }
  } else 
  if (!FullGrid && T>0) { // no prediction, try 1-context
    { // encode S
      auto It = ContextTS[CIdx].find(T);
      if (It == ContextTS[CIdx].end()) {
        auto Pair = ContextTS[CIdx].insert({T, one_context_type()});
        Pair.first->second[0] = 1;
        EncodeWithContext(T, 0, Pair.first->second.data(), &Coder);
        //EncodeCenteredMinimal(S, T+1, &BlockStream);
        EncodeUniform(T, R, &Coder);
        ++Pair.first->second[S+1];
      } else {
        It->second[0] = 1;
        if (It->second[S+1] == 0) {
          EncodeWithContext(T, 0, It->second.data(), &Coder);
          //EncodeCenteredMinimal(S, T+1, &BlockStream);
          EncodeUniform(T, S, &Coder);
        } else {
          EncodeWithContext(T, S+1, It->second.data(), &Coder);
        }
        ++It->second[S+1];
      }
    }
    { // encode R if necessary
      u32 CR = T*(ContextMax+2) + S;
      if (FullGrid) {
        assert(R == T-1);
      } else if (T==1 && S==1) {
        assert(R == 0);
      } else if (S == 0) {
        assert(R == T);
      } else {
        auto It = ContextTSR[CIdx].find(CR);
        if (It == ContextTSR[CIdx].end()) {
          auto Pair = ContextTSR[CIdx].insert({CR, one_context_type()});
          Pair.first->second[0] = 1;
          EncodeWithContext(T, 0, Pair.first->second.data(), &Coder); 
          //EncodeCenteredMinimal(R, T+1, &BlockStream);
          EncodeUniform(T, R, &Coder);
          ++Pair.first->second[R+1];
        } else {
          It->second[0] = 1;
          if (It->second[R+1] == 0) {
            EncodeWithContext(T, 0, It->second.data(), &Coder);
            //EncodeCenteredMinimal(R, T+1, &BlockStream);
            EncodeUniform(T, R, &Coder);
          } else {
            EncodeWithContext(T, R+1, It->second.data(), &Coder);
          }
          ++It->second[R+1];
        }
      }
    }
  }

#endif
  //SRList.push_back(vec2i{S, R});
  //++SRCounter;

  tree* SaveTreePtr = nullptr;
  if (Depth == Params.StartResolutionSplit) { // beginning of block
    SaveTreePtr = TreePtr;
    ++BlockCount;
    //REQUIRE(Split == ResolutionSplit);
    //FOR_EACH(Context, ContextS ) { Context->clear(); }
    //FOR_EACH(Context, ContextTS) { Context->clear(); }
    //FOR_EACH(Context, ContextR ) { Context->clear(); }
    static bool Done = false;
    if (!Done) {
      printf("grid dims is %d %d %d\n", Grid.Dims3.x, Grid.Dims3.y, Grid.Dims3.z);
      Done = true;
    }
  }

  /* recurse */
  tree* Left = nullptr; 
#if defined(LIGHT_PREDICT) || defined(TIME_PREDICT)
  if (S == 1) {
#elif defined(PREDICTION) || defined(RESOLUTION_PREDICT)
  if (S==1 && CellCountLeft==1) {
    assert(Depth+1 == Params.MaxDepth);
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  if (Begin+1 == Mid) {
#endif
    assert(Begin+1 == Mid);
#if defined(PREDICTION) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
    Left = new (TreePtr++) tree;
    Left->Count = 1;
    ++NumNodeAllocated;
#endif
    ++NParticlesDecoded;
    auto G = GridLeft;
    for (int DD = 0; DD < 3; ++DD) {
      while (G.Dims3[DD] > 1) {
        auto G1 = SplitGrid(G, DD, SpatialSplit, side::Left);
        auto G2 = SplitGrid(G, DD, SpatialSplit, side::Right);
        i32 M = Params.BBoxInt.Min[DD] + G2.From3[DD]*Params.W3[DD];
        bool Left = Particles[Begin].Pos[DD] < M;
        if (Left) G = G1; else G = G2;
        Write(&GlobalStream.Stream, Left);
      }
    }
    bbox_int BBox;
    BBox.Min = Params.BBoxInt.Min + G.From3*Params.W3;
    BBox.Max = Params.BBoxInt.Min + (G.From3+(G.Dims3-1)*G.Stride3+1)*Params.W3 - 1;
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Begin].Pos[DD] <= M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M+1;
        Write(&GlobalStream.Stream, Left);
      }
    }
#if defined(PREDICTION) || defined(LIGHT_PREDICT) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  } else if (S >= 1) { //recurse
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  } else if (Begin < Mid) {
#endif
    assert(Depth+1 < Params.MaxDepth);
#if defined(RESOLUTION_ALWAYS)
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
#else
    split_type NextSplit = 
      ((Depth+1==Params.StartResolutionSplit) ||
       (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
#endif
#if defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
    if (Split == SpatialSplit)
      Left = BuildTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Left = BuildTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl+1, Depth+1);
#else
    if (Split == SpatialSplit)
      Left = BuildTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Left = BuildTreeIntPredict(nullptr, Particles, Begin, Mid, S, GridLeft, NextSplit, ResLvl+1, Depth+1);
#endif
  }

  /* recurse on the right */
  tree* Right = nullptr;
#if defined(LIGHT_PREDICT) || defined(TIME_PREDICT)
  if (R == 1) {
#elif defined(PREDICTION) || defined(RESOLUTION_PREDICT)
  if (R==1 && CellCountRight==1) {
    assert(Depth+1 == Params.MaxDepth);
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL) || defined(FORCE_BINOMIAL)
  if (Mid+1 == End) {
#endif
    assert(Mid+1 == End);
#if defined(PREDICTION) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
    Right = new (TreePtr++) tree;
    Right->Count = 1;
    ++NumNodeAllocated;
#endif
    ++NParticlesDecoded;
    auto G = GridRight;
    for (int DD = 0; DD < 3; ++DD) {
      while (G.Dims3[DD] > 1) {
        auto G1 = SplitGrid(G, DD, SpatialSplit, side::Left);
        auto G2 = SplitGrid(G, DD, SpatialSplit, side::Right);
        i32 M = Params.BBoxInt.Min[DD] + G2.From3[DD]*Params.W3[DD];
        bool Left = Particles[Mid].Pos[DD] < M;
        if (Left) G = G1; else G = G2;
        Write(&GlobalStream.Stream, Left);
      }
    }
    bbox_int BBox;
    BBox.Min = Params.BBoxInt.Min + G.From3*Params.W3;
    BBox.Max = Params.BBoxInt.Min + (G.From3+(G.Dims3-1)*G.Stride3+1)*Params.W3 - 1;
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Mid].Pos[DD] <= M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M+1;
        Write(&GlobalStream.Stream, Left);
      }
    }
#if defined(PREDICTION) || defined(LIGHT_PREDICT) || defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  } else if (R >= 1) { //recurse
#elif defined(NORMAL) || defined(SOTA) || defined(BINOMIAL)
  } else if (Mid < End) {
#endif
    assert(Depth+1 < Params.MaxDepth);
#if defined(RESOLUTION_ALWAYS)
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
#else
    split_type NextSplit = (Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
#endif
#if defined(TIME_PREDICT)
    if (Split == SpatialSplit)
      Right = BuildTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, R, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Right = BuildTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, R, GridRight, NextSplit, ResLvl+1, Depth+1);
#elif defined(RESOLUTION_PREDICT)
    if (Split == SpatialSplit)
      Right = BuildTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, R, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Right = BuildTreeIntPredict(PredNode?PredNode->Right:Left, Particles, Mid, End, R, GridRight, NextSplit, ResLvl+1, Depth+1);
#else
    if (Split == SpatialSplit)
      Right = BuildTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, R, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      Right = BuildTreeIntPredict(Left, Particles, Mid, End, R, GridRight, NextSplit, ResLvl+1, Depth+1);
#endif
  }

  /* construct the prediction tree */
  tree* Node = nullptr; // TODO: try to move this to the beginning and see if that improves performance?
#if defined(PREDICTION)
  if (Split == ResolutionSplit) {
    Node = BuildPredTree(Left, Right, Depth, D);
  } else if (Split == SpatialSplit) {
    if (Depth > Params.StartResolutionSplit) {
      Node = new (TreePtr++) tree;
      Node->Left  = Left;
      Node->Right = Right;
      if (Left ) Node->Count = Left->Count; else Node->Count = 0;
      if (Right) Node->Count += Right->Count;
    }
  }
  if (Depth==Params.StartResolutionSplit && Node!=nullptr) {
    TreePtr = SaveTreePtr; // free memory
    *TreePtr = *Node;
    Node = TreePtr++;
  }
#elif defined(TIME_PREDICT) || defined(RESOLUTION_PREDICT)
  if (Left || Right) {
    Node = new (TreePtr++) tree;
    Node->Left = Left;
    Node->Right = Right;
    if (Left ) Node->Count = Left->Count; else Node->Count = 0;
    if (Right) Node->Count += Right->Count;
  }
  if (Depth==Params.StartResolutionSplit && Node!=nullptr) {
    TreePtr = SaveTreePtr; // free memory
    *TreePtr = *Node;
    Node = TreePtr++;
  }
#endif

  return Node;
}

/* we do not do "resolution splits" any more
* "Grid" refers to the grid made of blocks, not individual cells */
static u32 Stack[128] = {};
static i8 StackPtr = 0;
static constexpr i8 ContextLength = 1;
//static constexpr u32 ContextMax = 64;
//static u32 ContextGS[ContextMax+2][ContextMax+2] = {};
//static u32 ContextGR[ContextMax+2][ContextMax+2][ContextMax+2] = {}; // [N+1] is the ESC symbol
//static u32 ContextGRR[ContextMax+2][ContextMax+2] = {};

INLINE u64
GetBlockIndex(const vec3i& From3) {
  vec3i BlockId3 = From3 / Params.BlockDims3;
  u64 BlockIdx = u64(BlockId3.z)*Params.NBlocks3.x*Params.NBlocks3.y + u64(BlockId3.y)*Params.NBlocks3.x + u64(BlockId3.x);
  return BlockIdx;
}

// TODO: use the depth to determine whether we are inside a block or still above
static stream*
GetStream(const grid_int& Grid, i8 Depth) {
  if (Depth >= Params.StartResolutionSplit) {
    REQUIRE(Grid.Dims3.x <= Params.BlockDims3.x);
    REQUIRE(Grid.Dims3.y <= Params.BlockDims3.y);
    REQUIRE(Grid.Dims3.z <= Params.BlockDims3.z);
    u64 BlockIdx = GetBlockIndex(Grid.From3);
    REQUIRE(Streams.size() > BlockIdx);
    return &Streams[BlockIdx];
  }
  return &GlobalStream;
}

INLINE f64
GetScore(const grid_int& Grid, i64 N) {
  return f64(Grid.Dims3.x)*f64(Grid.Dims3.y)*f64(Grid.Dims3.z);
  //return f64(Grid.Dims3.x)*f64(Grid.Dims3.y)*f64(Grid.Dims3.z)/f64(N);
  //return 1000000;
}

using stack = std::vector<q_item_int>;
struct heap_priority {
  //f32 Error = 0;
  i8 Level = 0;
  i32 NParticles = 0; // num particles left so far on the current level
  i32 TotalNParticlesOnLevel = 0; // total of particles on the current level
  //i64 TotalNParticles = 0; // total of particles on the current level
  //i64 ParticleCount = 0; // number of particles read so far
  stream* Stream = nullptr;
  stack* Stack = nullptr;
  i64 BlockId = 0;
  INLINE bool operator<(const heap_priority& Other) const { 
    if (Level == Other.Level) {
      if (f64(NParticles)/f64(TotalNParticlesOnLevel) == f64(Other.NParticles)/f64(Other.TotalNParticlesOnLevel)){
      //if (NParticles == Other.NParticles) {
        return Stream < Other.Stream;
      }
      return f64(NParticles)/f64(TotalNParticlesOnLevel) < f64(Other.NParticles)/f64(Other.TotalNParticlesOnLevel);
      //return (NParticles) < (Other.NParticles);
      //return f64(ParticleCount)/f64(TotalNParticles) > f64(Other.ParticleCount)/f64(TotalNParticles);
    }
    return Level < Other.Level;
  };
};
struct heap_data {
  i64 BlockId = 0;
  stream* Stream = nullptr;
  stack* Stack = nullptr;
  INLINE bool operator<(const heap_data& Other) const { return Stream < Other.Stream; }
};

static int BitCount = 0;
static FILE* FilePtr = nullptr;
static bitstream MetaStream;
static i64 StreamSize = 0;

static bool IsPowerOfTwo(i64 x) { return (x & (x - 1)) == 0; }

static i64
WriteBlock(FILE* Fp, stream* S, bitstream* Bs) {
  i64 AddedBytes = 0;
  { // stream
    S->StreamSize = (u32)Size(S->Stream);
    if (S->StreamSize > 0) {
      Flush(&S->Stream);
      fwrite(S->Stream.Stream.Data, S->StreamSize, 1, Fp);
      AddedBytes += S->StreamSize;
    }
    DeallocBuf(&S->Stream.Stream);
    //std::cout << S->StreamSize << "\n";
  }
  { // coder
    S->CoderSize = (u32)Size(S->Coder.BitStream);
    if (S->CoderSize > 0) {
      S->Coder.EncodeFinalize();
      fwrite(S->Coder.BitStream.Stream.Data, S->CoderSize, 1, Fp);
      AddedBytes += S->CoderSize;
    }
    DeallocBuf(&S->Coder.BitStream.Stream);
  }
  return AddedBytes;
}

static u32 BlockIndex = 0;
static void
BuildIntAdaptiveDFSPhase(
  std::vector<particle_int>& Particles, i64 Begin, i64 End, 
  const grid_int& Grid, split_type Split, i8 ResLvl, i8 Depth) 
{
  i8 D = Params.DimsStr[Depth] - 'x';
  i64 N = End - Begin;
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  auto Stream = GetStream(Grid, Depth);
  if (CellCount==1) {
    const auto& G = Grid;
    auto BlockIdx = GetBlockIndex(Grid.From3);
    auto& Block = OutputBlocks[BlockIdx];
    bbox_int BBox {
      .Min = Params.BBoxInt.Min + G.From3*Params.W3,
      .Max = BBox.Min + Params.W3
    };
    for (int DD = 0; DD < Params.NDims; ++DD) {
      REQUIRE(Particles[Begin].Pos[DD] >= BBox.Min[DD]);
      REQUIRE(Particles[Begin].Pos[DD] < BBox.Max[DD]);
    }
    //Block.BBoxesAndIds.push_back({Begin, BBox});
    REQUIRE(Block.BBoxesAndIds.size() <= Block.NParticles);
    // check if all particles have been seen
    if (Block.BBoxesAndIds.size() == Block.NParticles) { // now encode the rest of the refinement bits
      //i32 NParticlesEncoded = 0;
      //while (NParticlesEncoded < Block.NParticles) {
      //  auto W3 = Block.BBoxesAndIds[0].BBox.Max - Block.BBoxesAndIds[0].BBox.Min;
      //  i8 DD = 0;
      //  if (W3.y > W3[DD]) DD = 1;
      //  if (W3.z > W3[DD]) DD = 2;
      //  if (W3[DD] > 1) {
      //    FOR_EACH(B, Block.BBoxesAndIds) {
      //      assert(B->BBox.Max[DD]-B->BBox.Min[DD] == W3[DD]);
      //      i32 M = (B->BBox.Max[DD]+B->BBox.Min[DD]) >> 1;
      //      bool Left = Particles[B->Id].Pos[DD] < M;
      //      auto BBoxCopy = B->BBox;
      //      if (Left) B->BBox.Max[DD] = M; else B->BBox.Min[DD] = M;
      //      assert(IsPowerOfTwo(B->BBox.Max[DD]-B->BBox.Min[DD]));
      //      GrowIfTooFull(&Stream->Stream);
      //      Write(&Stream->Stream, Left);
      //      if (B->BBox.Min+1 == B->BBox.Max) {
      //        ++NParticlesEncoded; 
      //      }
      //      ++Block.BitCount;
      //    }
      //    if (NParticlesEncoded == Block.NParticles) {
      //      printf("block %lld bitcount %d\n", BlockIdx, Block.BitCount);
      //    }
      //  } else {
      //    NParticlesEncoded = Block.NParticles;
      //    printf("- block %lld bitcount %d\n", BlockIdx, Block.BitCount);
      //  }
      //}
      // block is done, release memory
      Block.BBoxesAndIds.clear();
      //WriteBlock(FilePtr, );
    }
    return;
  } else if (CellCount > 1) {
    i64 Mid = Begin;
    i32 MM = Grid.From3[D];
    if (Split == ResolutionSplit) {
      auto RPred = [D, &Grid](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        Bin = (Bin-Grid.From3[D]) / Grid.Stride3[D];
        return IS_EVEN(Bin);
      };
      Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
    } else if (Split == SpatialSplit) {
      MM = Grid.From3[D] + (((Grid.Dims3[D]+1)>>1)-1) * Grid.Stride3[D];
      auto SPred = [MM, D, &Grid](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        return Bin <= MM;
      };
      Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
    }
    auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
    auto GridRight = SplitGrid(Grid, D, Split, side::Right);
    i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
    i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
    REQUIRE(CellCountLeft+CellCountRight == CellCount);
    i64 P = Mid - Begin;
    if (CellCount-N < N) {
      N = CellCount - N;
      P = CellCountLeft - P;
    }
    GrowIfTooFull(&Stream->Stream);
  #if defined(FORCE_BINOMIAL)
    f64 Mean = f64(N) / 2; // mean
    f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
    GrowIfTooFull(&Stream->Coder.BitStream);
    EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &Stream->Stream, &Stream->Coder);
  #else
    EncodeCenteredMinimal(u32(P), u32(N+1), &Stream->Stream);
  #endif
    if (CellCountLeft>=1 && Begin+1<=Mid) {
      split_type NextSplit = 
            ((Depth+1==Params.StartResolutionSplit) ||
             (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
      BuildIntAdaptiveDFSPhase(Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
    }
    if (CellCountRight>=1 && Mid+1<=End) {
      split_type NextSplit = SpatialSplit;
      BuildIntAdaptiveDFSPhase(Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    }
  }
}

// TODO: for the arithmetic streams, write at least 32 bits (4 bytes) to avoid reading past the stream
static void
BuildIntAdaptiveBFSPhase(std::vector<particle_int>& Particles, std::queue<q_item_int>& Queue) {
  while (!Queue.empty()) {
    auto Q = Queue.front();
    Queue.pop();
    i64 N = Q.End - Q.Begin;
    i64 Mid = Q.Begin;
    i8 D = Params.DimsStr[Q.Depth] - 'x';
    i64 CellCount = i64(Q.Grid.Dims3.x) * i64(Q.Grid.Dims3.y) * i64(Q.Grid.Dims3.z);
    i32 MM = Q.Grid.From3[D];
    if (Q.Split == ResolutionSplit) {
      auto RPred = [&Q, D](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        Bin = (Bin-Q.Grid.From3[D]) / Q.Grid.Stride3[D];
        return IS_EVEN(Bin);
      };
      Mid = std::partition(RANGE(Particles, Q.Begin, Q.End), RPred) - Particles.begin();
    } else if (Q.Split == SpatialSplit) {
      MM = Q.Grid.From3[D] + (((Q.Grid.Dims3[D]+1)>>1)-1) * Q.Grid.Stride3[D];
      auto SPred = [&Q, MM, D](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        return Bin <= MM;
      };
      Mid = std::partition(RANGE(Particles, Q.Begin, Q.End), SPred) - Particles.begin();
    }
    auto GridLeft  = SplitGrid(Q.Grid, D, Q.Split, side::Left );
    auto GridRight = SplitGrid(Q.Grid, D, Q.Split, side::Right);
    i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
    i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
    i64 P = Mid - Q.Begin;
    //printf("P = %lld\n", P);
    if (CellCount-N < N) {
      N = CellCount - N;
      P = CellCountLeft - P;
    }
    auto Stream = GetStream(Q.Grid, Q.Depth);
#if defined(FORCE_BINOMIAL)
      f64 Mean = f64(N) / 2; // mean
      f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
      GrowIfTooFull(&Stream->Coder.BitStream);
      EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &Stream->Stream, &Stream->Coder);
#else
      GrowIfTooFull(&Stream->Stream);
      EncodeCenteredMinimal(u32(P), u32(N+1), &Stream->Stream);
#endif
    if (CellCountLeft==1) {
      REQUIRE(false);
    } else if (CellCountLeft>=1 && Q.Begin<Mid) {
      split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
      q_item_int Next{
        .Begin = Q.Begin,
        .End = Mid,
        .Grid = GridLeft,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth+1),
        .Split = NextSplit
      };
      if (Next.Depth < Params.StartResolutionSplit) {
        Queue.push(Next);
      } else {
        //printf("%lld: %lld %lld\n", BlockCount, Next.Begin, Next.End);
        auto BlockIdx = GetBlockIndex(Next.Grid.From3);
        auto& Block = OutputBlocks[BlockIdx];
        Block.NParticles = i32(Next.End - Next.Begin);
        Block.BBoxesAndIds.reserve(Block.NParticles);
        BuildIntAdaptiveDFSPhase(Particles, Next.Begin, Next.End, Next.Grid, Next.Split, Next.ResLvl, Next.Depth);
        ++BlockCount;
      }
    }

    if (CellCountRight==1) {
      REQUIRE(false);
    } else if (CellCountRight>=1 && Mid<Q.End) {
      split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
      q_item_int Next{
        .Begin = Mid,
        .End = Q.End,
        .Grid = GridRight,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth+1),
        .Split = NextSplit
      };
      if (Next.Depth < Params.StartResolutionSplit) {
        Queue.push(Next);
      } else {
        //printf("%lld: %lld %lld\n", BlockCount, Next.Begin, Next.End);
        auto BlockIdx = GetBlockIndex(Next.Grid.From3);
        auto& Block = OutputBlocks[BlockIdx];
        Block.NParticles = i32(Next.End - Next.Begin);
        Block.BBoxesAndIds.reserve(Block.NParticles);
        BuildIntAdaptiveDFSPhase(Particles, Next.Begin, Next.End, Next.Grid, Next.Split, Next.ResLvl, Next.Depth);
        ++BlockCount;
      }
    }
  }
}

static void
BuildIntAdaptive(std::vector<particle_int>& Particles, q_item_int Q) {
  if (Params.StartResolutionSplit > 0) {
    std::queue<q_item_int> Queue;
    Queue.push(Q);
    BuildIntAdaptiveBFSPhase(Particles, Queue);
  } else {
    auto BlockIdx = GetBlockIndex(Q.Grid.From3);
    auto& Block = OutputBlocks[BlockIdx];
    Block.NParticles = i32(Q.End - Q.Begin);
    Block.BBoxes.reserve(Block.NParticles);
    BuildIntAdaptiveDFSPhase(Particles, Q.Begin, Q.End, Q.Grid, Q.Split, Q.ResLvl, Q.Depth);
  }
}

static block_info MyBlock;

static void
BuildTreeIntBFS(q_item_int Q, std::vector<particle_int>& Particles) {
  printf("------BFS builder------\n");
  std::queue<q_item_int> Queue;
  Queue.push(Q);
  while (!Queue.empty()) {
    Q = Queue.front();
    Queue.pop();
    i64 N = Q.End - Q.Begin;
    i64 Mid = Q.Begin;
    i8 D = Params.DimsStr[Q.Depth] - 'x';
    i64 CellCount = i64(Q.Grid.Dims3.x) * i64(Q.Grid.Dims3.y) * i64(Q.Grid.Dims3.z);
    i32 MM = Q.Grid.From3[D];
    if (Q.Split == ResolutionSplit) {
      auto RPred = [&Q, D](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        Bin = (Bin-Q.Grid.From3[D]) / Q.Grid.Stride3[D];
        return IS_EVEN(Bin);
      };
      Mid = std::partition(RANGE(Particles, Q.Begin, Q.End), RPred) - Particles.begin();
    } else if (Q.Split == SpatialSplit) {
      MM = Q.Grid.From3[D] + (((Q.Grid.Dims3[D]+1)>>1)-1) * Q.Grid.Stride3[D];
      auto SPred = [&Q, MM, D](const particle_int& P) {
        i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
        return Bin <= MM;
      };
      Mid = std::partition(RANGE(Particles, Q.Begin, Q.End), SPred) - Particles.begin();
    }
    auto GridLeft  = SplitGrid(Q.Grid, D, Q.Split, side::Left );
    auto GridRight = SplitGrid(Q.Grid, D, Q.Split, side::Right);
    i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
    i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
    i64 P = Mid - Q.Begin;
    if (CellCount-N < N) {
      N = CellCount - N;
      P = CellCountLeft - P;
    }
    auto& Stream = GlobalStream;
    GrowIfTooFull(&Stream.Stream);
    EncodeCenteredMinimal(u32(P), u32(N+1), &Stream.Stream);
    if (Q.Begin+1==Mid && CellCountLeft==1) {
    } else if (Q.Begin < Mid) {
      split_type NextSplit = 
        ((Q.Depth+1==Params.StartResolutionSplit) ||
         (Q.Split==ResolutionSplit && Q.ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
      Queue.push(q_item_int{
        .Begin = Q.Begin,
        .End = Mid,
        .Grid = GridLeft,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth + 1),
        .Split = NextSplit
      });
    }

    if (Mid+1==Q.End && CellCountRight==1) {
    } else if (Mid < Q.End) {
      split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
      Queue.push(q_item_int{
        .Begin = Mid,
        .End = Q.End,
        .Grid = GridRight,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth + 1),
        .Split = NextSplit
      });
    }
  }
}

#define IN_THE_CUT \
  InTheCut = BitCount < Params.DecodeBudget*8;\
  //InTheCut = Q.Depth <= 19; \

static void
DecodeTreeIntBFS(q_item_int Q) {
  printf("------BFS decoder------\n");
  std::queue<q_item_int> Queue;
  Queue.push(Q);
  bool InTheCut = true;
  while (!Queue.empty()) {
    Q = Queue.front();
    Queue.pop();
    i64 N = Q.End - Q.Begin;
    i64 Mid = Q.Begin;
    i8 D = Params.DimsStr[Q.Depth] - 'x';
    auto GridLeft  = SplitGrid(Q.Grid, D, Q.Split, side::Left );
    auto GridRight = SplitGrid(Q.Grid, D, Q.Split, side::Right);
    i64 CellCount = i64(Q.Grid.Dims3.x) * i64(Q.Grid.Dims3.y) * i64(Q.Grid.Dims3.z);
    i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
    i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
    InTheCut = BitCount<Params.DecodeBudget*8/* && Q.Depth<=Params.DecodeDepth*/;
    auto& Stream = GlobalStream;
    bool Flip = CellCount-N < N;
    if (Flip) N = CellCount - N;
    if (InTheCut) {
      u32 P = 0;
      BitCount += DecodeCenteredMinimal(u32(N+1), &Stream.Stream, P);
      InTheCut = BitCount<Params.DecodeBudget*8;
      if (Flip) { P = u32(CellCountLeft-P); N = CellCount-N; }
      Mid = P + Q.Begin;
    } else { // out of the cut
      if (Flip) { N = CellCount - N; }
      NParticlesDecoded += 1;
      GenerateParticlesPerNode(1, Q.Grid, &OutputParticles);
      continue;
    }
    /* ---------------------- LEFT ----------------- */
    if (InTheCut && Q.Begin+1==Mid && CellCountLeft==1) {
      NParticlesDecoded += 1;
      GenerateParticlesPerNode(1, GridLeft, &OutputParticles);
    } else if (InTheCut && Q.Begin<Mid) { // recurse
      split_type NextSplit = 
        ((Q.Depth+1==Params.StartResolutionSplit) ||
         (Q.Split==ResolutionSplit && Q.ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
      Queue.push(q_item_int{
        .Begin = Q.Begin,
        .End = Mid,
        .Grid = GridLeft,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth + 1),
        .Split = NextSplit
      });
    }
    /* ---------------------- RIGHT ----------------- */
    if (InTheCut && Mid+1==Q.End && CellCountRight==1) {
      NParticlesDecoded += 1;
      GenerateParticlesPerNode(1, GridRight, &OutputParticles);
    } else if (InTheCut && Mid<Q.End) { // recurse
      split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
      Queue.push(q_item_int{
        .Begin = Mid,
        .End = Q.End,
        .Grid = GridRight,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth + 1),
        .Split = NextSplit
      });
    }
  } // end of loop
}

static void
BuildTreeIntDFS(std::vector<particle_int>& Particles, i64 Begin, i64 End, 
  const grid_int& Grid, split_type Split, i8 ResLvl, i8 Depth) {
  i8 D = Params.DimsStr[Depth] - 'x';
  i64 N = End - Begin;
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  i64 Mid = Begin;
  i32 MM = Grid.From3[D];
  if (Split == ResolutionSplit) {
    auto RPred = [D, &Grid](const particle_int& P) {
      i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
      Bin = (Bin-Grid.From3[D]) / Grid.Stride3[D];
      return IS_EVEN(Bin);
    };
    Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else if (Split == SpatialSplit) {
    MM = Grid.From3[D] + (((Grid.Dims3[D]+1)>>1)-1) * Grid.Stride3[D];
    auto SPred = [MM, D, &Grid](const particle_int& P) {
      i32 Bin = (P.Pos[D]-Params.BBoxInt.Min[D]) / Params.W3[D];
      return Bin <= MM;
    };
    Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }
  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  REQUIRE(CellCountLeft+CellCountRight == CellCount);
  i64 P = Mid - Begin;
  if (CellCount-N < N) {
    N = CellCount - N;
    P = CellCountLeft - P;
  }
  N = MIN(N, CellCountRight); // this only makes sense if the grid dimension is non power of two (so that the right can have fewer cells than the left)
  auto& Stream = GlobalStream;
#if defined(FORCE_BINOMIAL)
    f64 Mean = f64(N) / 2; // mean
    f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
    EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &Stream.Stream, &Stream.Coder);
#else
  EncodeCenteredMinimal(u32(P), u32(N+1), &Stream.Stream);
#endif
  if (Begin+1==Mid && CellCountLeft==1) {
    const auto& G = GridLeft;
    bbox_int BBox {
      .Min = Params.BBoxInt.Min + G.From3*Params.W3,
      .Max = BBox.Min + Params.W3
    };
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]+1) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Begin].Pos[DD] < M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        GrowIfTooFull(&Stream.Stream);
        Write(&Stream.Stream, Left);
      }
    }
  } else if (Begin < Mid) {
    split_type NextSplit = 
      ((Depth+1==Params.StartResolutionSplit) ||
       (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      BuildTreeIntDFS(Particles, Begin, Mid, GridLeft, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      BuildTreeIntDFS(Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
  }
  if (Mid+1==End && CellCountRight==1) {
    const auto& G = GridRight;
    bbox_int BBox {
      .Min = Params.BBoxInt.Min + G.From3*Params.W3,
      .Max = BBox.Min + Params.W3
    };
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]+1) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Mid].Pos[DD] < M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        Write(&Stream.Stream, Left);
      }
    }
  } else if (Mid < End) {
    split_type NextSplit = (Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      BuildTreeIntDFS(Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      BuildTreeIntDFS(Particles, Mid, End, GridRight, NextSplit, ResLvl+1, Depth+1);
  }
}

static void
DecodeIntAdaptiveBFSPhase(std::queue<q_item_int>& Queue, std::priority_queue<heap_priority>& Heap, std::vector<stack>& Stacks) {
  bool InTheCut = BitCount < Params.DecodeBudget*8;
  while (!Queue.empty()) {
    auto Q = Queue.front();
    Queue.pop();
    i64 N = Q.End - Q.Begin;
    i64 Mid = Q.Begin;
    i8 D = Params.DimsStr[Q.Depth] - 'x';
    auto GridLeft  = SplitGrid(Q.Grid, D, Q.Split, side::Left );
    auto GridRight = SplitGrid(Q.Grid, D, Q.Split, side::Right);
    i64 CellCount = i64(Q.Grid.Dims3.x) * i64(Q.Grid.Dims3.y) * i64(Q.Grid.Dims3.z);
    i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
    i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
    InTheCut = BitCount < Params.DecodeBudget*8;
    auto Stream = GetStream(Q.Grid, Q.Depth);
    if (InTheCut) {
      bool Flip = CellCount-N < N;
      if (Flip) N = CellCount - N;
      u32 P = 0;
#if defined(FORCE_BINOMIAL)
      f64 Mean = f64(N) / 2; // mean
      f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
      BitCount += DecodeRange(Mean, StdDev, f64(0), f64(N), CdfTable, &Stream->Stream, &Stream->Coder, P);
#else
      BitCount += DecodeCenteredMinimal(u32(N+1), &Stream->Stream, P);
#endif
      if (Flip) { P = (u32)(CellCountLeft-P); N = CellCount-N; }
      Mid = P + Q.Begin;
      //printf("P = %u\n", P);
    } else { // do not read any more bits, just generate particles
      NParticlesDecoded += N;
      GenerateParticlesPerNode(N, Q.Grid, &OutputParticles);
      continue;
    }
    InTheCut = BitCount < Params.DecodeBudget*8;
    if (InTheCut && CellCountLeft==1) {
      REQUIRE(false); // should not happen
    } else if (InTheCut && CellCountLeft>=1 && Q.Begin<Mid) {
      split_type NextSplit = 
        ((Q.Depth+1==Params.StartResolutionSplit) ||
         (Q.Split==ResolutionSplit && Q.ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
        q_item_int Next{
          .Begin = Q.Begin,
          .End = Mid,
          .Grid = GridLeft,
          .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
          .Depth = i8(Q.Depth+1),
          .Split = NextSplit
        };
      if (Q.Depth+1 < Params.StartResolutionSplit) {
        Queue.push(Next);
      } else {
        //printf("%lld: %lld %lld\n", BlockCount, Next.Begin, Next.End);
        i64 BlockIdx = GetBlockIndex(Next.Grid.From3);
        auto& Block = OutputBlocks[BlockIdx];
        Block.NParticles = i32(Next.End - Next.Begin);
        Block.BBoxes.reserve(Block.NParticles);
        Stacks[BlockIdx].push_back(Next);
        Heap.push(
          heap_priority{
            .Level=100, 
            .NParticles=i32(Next.End-Next.Begin),
            .TotalNParticlesOnLevel=i32(Next.End-Next.Begin),
            .Stream=&Streams[BlockIdx],
            .Stack=&Stacks[BlockIdx],
            .BlockId=BlockIdx});
        ++BlockCount;
      }
    }
    InTheCut = BitCount < Params.DecodeBudget*8;
    if (InTheCut && CellCountRight==1) {
      REQUIRE(false);
    } else if (InTheCut && CellCountRight>=1 && Mid<Q.End) {
      split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
      q_item_int Next{
        .Begin = Mid,
        .End = Q.End,
        .Grid = GridRight,
        .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
        .Depth = i8(Q.Depth+1),
        .Split = NextSplit
      };
      if (Q.Depth+1 < Params.StartResolutionSplit) {
        Queue.push(Next);
      } else {
        //printf("%lld: %lld %lld\n", BlockCount, Next.Begin, Next.End);
        i64 BlockIdx = GetBlockIndex(Next.Grid.From3);
        auto& Block = OutputBlocks[BlockIdx];
        Block.NParticles = i32(Next.End - Next.Begin);
        Block.BBoxes.reserve(Block.NParticles);
        Stacks[BlockIdx].push_back(Next);
        Heap.push(heap_priority{
            .Level=100, 
            .NParticles=i32(Next.End-Next.Begin), 
            .TotalNParticlesOnLevel=i32(Next.End-Next.Begin), 
            .Stream=&Streams[BlockIdx],
            .Stack=&Stacks[BlockIdx],
            .BlockId=BlockIdx});
        ++BlockCount;
      }
    }
  }
}

static int
DecodeIntAdaptiveDFSPhase(heap_priority& TopPriority) {
  auto Stack = TopPriority.Stack;
  bool InTheCut = BitCount < Params.DecodeBudget*8;
  int PCount = 0;
  auto BitCountBackup = BitCount;
  while (InTheCut && !Stack->empty() && BitCount-BitCountBackup<512) {
    auto Q = Stack->back();
    auto Stream = GetStream(Q.Grid, Q.Depth);
    if (Q.ResLvl != TopPriority.Level) { // TODO: move the update out of the loop
      TopPriority.Level = Q.ResLvl;
      TopPriority.NParticles = i32(Q.End - Q.Begin);
      TopPriority.TotalNParticlesOnLevel = i32(Q.End - Q.Begin);
    }
    i8 D = Params.DimsStr[Q.Depth] - 'x';
    assert((Q.Grid.Dims3[D]&1) == 0);
    i64 N = Q.End - Q.Begin;
    i64 CellCount = i64(Q.Grid.Dims3.x) * i64(Q.Grid.Dims3.y) * i64(Q.Grid.Dims3.z);
    assert(CellCount > 0);
    i64 Mid = Q.Begin;
    auto BlockIdx = GetBlockIndex(Q.Grid.From3);
    //assert(BlockIdx == TopPriority.BlockId);
    if (CellCount==1) {
      block_info& Block = OutputBlocks[BlockIdx];
      const auto& G = Q.Grid;
      bbox_int BBox {
        .Min = Params.BBoxInt.Min + G.From3*Params.W3,
        .Max = BBox.Min + Params.W3
      };
      Block.BBoxes.push_back(BBox);
      Block.ParticleCount.push_back(u32(N));
      //printf("N = %d\n", N);
      REQUIRE(Block.BBoxes.size() <= Block.NParticles);
      Stack->pop_back();
      continue;
    } else if (CellCount > 1) {
      InTheCut = BitCount < Params.DecodeBudget*8;
      auto GridLeft  = SplitGrid(Q.Grid, D, Q.Split, side::Left );
      auto GridRight = SplitGrid(Q.Grid, D, Q.Split, side::Right);
      i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
      i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
      assert(CellCountLeft>0 && CellCountRight>0);
      assert(CellCountLeft == CellCountRight);
      REQUIRE(CellCountLeft+CellCountRight == CellCount);
      Stack->pop_back();
      if (InTheCut) {
        bool Flip = CellCount-N < N;
        if (Flip) N = CellCount - N;
        u32 P = 0; // TODO: make it possible to allow P to be 64 bits
    #if defined(FORCE_BINOMIAL)
        if (Q.Split == ResolutionSplit) {
          f64 Mean = f64(N) / 2; // mean
          f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
          BitCount += DecodeRange(Mean, StdDev, f64(0), f64(N), CdfTable, &Stream->Stream, &Stream->Coder, P);
        } else {
          BitCount += DecodeCenteredMinimal(u32(N+1), &Stream->Stream, P);
        }
    #else
        BitCount += DecodeCenteredMinimal(u32(N+1), &Stream->Stream, P);
    #endif
        if (Flip) P = u32(CellCountLeft-P);
        Mid = P + Q.Begin;
      } else { // stop going down in this branch
        continue;
      }
      InTheCut = BitCount < Params.DecodeBudget*8;
      if (InTheCut && CellCountLeft>=1 && Mid+1<=Q.End) {
        split_type NextSplit = (Q.Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
        q_item_int Next {
          .Begin = Mid,
          .End = Q.End,
          .Grid = GridRight,
          .ResLvl = Q.ResLvl+ (Q.Split==ResolutionSplit),
          .Depth = Q.Depth+1,
          .Split = NextSplit
        };
        Stack->push_back(Next);
      }
      if (InTheCut && CellCountRight>=1 && Q.Begin+1<=Mid) {
        split_type NextSplit = 
          ((Q.Depth+1==Params.StartResolutionSplit) ||
           (Q.Split==ResolutionSplit && Q.ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
        q_item_int Next {
          .Begin = Q.Begin,
          .End = Mid,
          .Grid = GridLeft,
          .ResLvl = Q.ResLvl + (Q.Split==ResolutionSplit),
          .Depth = Q.Depth+1,
          .Split = NextSplit
        };
        Stack->push_back(Next);
      }
    }
  }
  return PCount;
}

// TODO: reserve memory in advance
// TODO: allocate a large memory arena upfront for the bit streams
// TODO: we can't simply decode the right particle before going all the way to the left because it may belong to the next resolution
static void
DecodeIntAdaptive(q_item_int Q) {
  printf("------Adaptive decoder------\n");
  // TODO: resize the Stacks to the number of blocks
  std::queue<q_item_int> Queue;
  std::priority_queue<heap_priority> Heap;
  std::vector<stack> Stacks; // one stack for each block
  Stacks.resize(u64(Params.NBlocks3.x)*u64(Params.NBlocks3.y)*u64(Params.NBlocks3.z));
  Queue.push(Q);
  /*-------------------- BFS phase ---------------------- */
  if (Params.StartResolutionSplit > 0) {
    DecodeIntAdaptiveBFSPhase(Queue, Heap, Stacks);
  } else {
    i64 BlockIdx = GetBlockIndex(Q.Grid.From3);
    auto& Block = OutputBlocks[BlockIdx];
    Block.NParticles = i32(Q.End - Q.Begin);
    Block.BBoxes.reserve(Block.NParticles);
    Stacks[BlockIdx].push_back(Q);
    Heap.push(heap_priority{
      .Level=100, 
      .NParticles=i32(Q.End-Q.Begin), 
      .TotalNParticlesOnLevel=i32(Q.End-Q.Begin), 
      .Stream=&Streams[BlockIdx],
      .Stack=&Stacks[BlockIdx],
      .BlockId=BlockIdx});
  }
  /*-------------------- DFS phase ---------------------- */
  bool InTheCut = BitCount < Params.DecodeBudget*8;
  std::vector<i64> BlockCount(Stacks.size(), 0);
  // TODO: we need to prioritize blocks where particles are less refined
  while (InTheCut && !Heap.empty()) {
    heap_priority TopPriority = Heap.top();
    auto& Block = OutputBlocks[TopPriority.BlockId];
    if (Block.BBoxes.size() == Block.NParticles) { // every particle has been seen once
      if (!Block.AtRefinement) {
        Block.AtRefinement = true;
        TopPriority.Level = -1;
      }
      auto W3 = Block.BBoxes[0].Max - Block.BBoxes[0].Min;
      i8 DD = 0;
      if (W3.y > W3[DD]) DD = 1;
      if (W3.z > W3[DD]) DD = 2;
      auto& Stream = Streams[TopPriority.BlockId];
      FOR_EACH(B, Block.BBoxes) {
        if (W3[DD] > 1) {
          bool Left = Read(&Stream.Stream);
          ++Block.BitCount;
          ++BitCount;
          InTheCut = BitCount < Params.DecodeBudget*8;
          if (!InTheCut)
            break;
          i32 M = (B->Max[DD]+B->Min[DD]) >> 1;
          if (Left) B->Max[DD] = M; else B->Min[DD] = M;
        }
        if (B->Min+1 == B->Max) {
          ++Block.NParticlesDecoded;
        }
      }
      ++TopPriority.Level;
      // TODO: need to update the priority so that the same block does not pop up multiple times
      REQUIRE(Block.NParticlesDecoded <= Block.NParticles);
      Heap.pop();
      if (Block.NParticlesDecoded != Block.NParticles) {
        Heap.push(TopPriority);
      }
    } else { // still in the DFS phase, not refinement
      Heap.pop();
      auto PCount = DecodeIntAdaptiveDFSPhase(TopPriority);
      const auto& Stack = *(TopPriority.Stack);
      Heap.push(TopPriority);
    }
    InTheCut = BitCount < Params.DecodeBudget*8;
  }
  //printf("heap size = %d\n", Heap.size());
  // generate the particles
  FOR_EACH(B, OutputBlocks) {
    printf("n %d ndecoded %d bboxes %lld\n", B->NParticles, B->NParticlesDecoded, B->BBoxes.size());
    for (size_t I = 0; I < B->BBoxes.size(); ++I) {
      vec3i Dims3 = B->BBoxes[I].Max - B->BBoxes[I].Min + 1;
      for (u32 J = 0; J < B->ParticleCount[I]; ++J) {
        i32 X = rand() % Dims3.x;
        i32 Y = rand() % Dims3.y;
        i32 Z = rand() % Dims3.z;
        vec3i P3 = B->BBoxes[I].Min + vec3i(X, Y, Z);
        OutputParticles.push_back(particle_int{.Pos=P3});
        // TODO: here we don't really care if we have duplicated points
      }
      NParticlesDecoded += B->ParticleCount[I];
    }
  }
}

static tree*
DecodeTreeIntDFS(const tree* PredNode, i64 Begin, i64 End, const grid_int& Grid, split_type Split, i8 ResLvl, i8 Depth) {
  i8 D = Params.DimsStr[Depth] - 'x';
  i64 N = End - Begin;
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  assert(N <= CellCount);
  i64 Mid = Begin;
  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  bool InTheCut = BitCount < Params.DecodeBudget*8;
  u32 P = 0;
  bool Flip = CellCount-N < N;
  if (Flip) N = CellCount - N;
  auto& Stream = GlobalStream;
  if (InTheCut) {
#if defined(FORCE_BINOMIAL)
    f64 Mean = f64(N) / 2; // mean
    f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
    BitCount += DecodeRange(Mean, StdDev, f64(0), f64(N), CdfTable, &Stream.Stream, &Stream.Coder, P);
#else
    BitCount += DecodeCenteredMinimal(u32(N+1), &Stream.Stream, P);
#endif
  } else { // out of bit budget, we need to predict P
    return nullptr;
    //if (PredNode) {
    //  i64 M = PredNode->Count;
    //  i64 K = PredNode->Left?PredNode->Left->Count : M - PredNode->Right->Count;
    //  P = N * K / M;  // P <= N
    //  P = MIN(P, CellCountLeft);
    //} else {
    //  return nullptr;
    //}
  }
  if (Flip) P = u32(CellCountLeft-P);
  assert(P <= CellCountLeft);
  Mid = P + Begin;

  /* ----------------- LEFT --------------- */
  InTheCut = BitCount < Params.DecodeBudget*8;
  REQUIRE(CellCountLeft+CellCountRight == CellCount);
  tree* LeftTree = nullptr;
  i64 NParticlesLeft = 0;
  if (InTheCut && Begin+1==Mid && CellCountLeft==1) { // one particle on the left
    const auto& G = GridLeft;
    bbox_int BBox {
      .Min = Params.BBoxInt.Min + G.From3*Params.W3,
      .Max = BBox.Min + Params.W3
    };
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]+1) {
        if (BitCount < Params.DecodeBudget*8)  {
          bool Left = Read(&Stream.Stream);
          ++BitCount;
          i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
          if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        } else {
          InTheCut = false;
          goto GENERATE_PARTICLE_LEFT;
        }
      }
    }
  GENERATE_PARTICLE_LEFT:
    OutputParticles.push_back(particle_int{.Pos=BBox.Min});
    ++NParticlesDecoded;
    //LeftTree = new (TreePtr++) tree;
    //LeftTree->Count = 1;
  } else if (InTheCut && Begin<Mid) {
    split_type NextSplit = 
      ((Depth+1==Params.StartResolutionSplit) ||
       (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      LeftTree = DecodeTreeIntDFS(PredNode?PredNode->Left:nullptr, Begin, Mid, GridLeft, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      LeftTree = DecodeTreeIntDFS(nullptr, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
  }

  /* ----------------- RIGHT --------------- */
  tree* RightTree = nullptr;
  InTheCut = BitCount < Params.DecodeBudget*8;
  if (InTheCut && Mid+1==End && CellCountRight==1) {
    const auto& G = GridRight;
    bbox_int BBox {
      .Min = Params.BBoxInt.Min + G.From3*Params.W3,
      .Max = BBox.Min + Params.W3
    };
    for (int DD = 0; DD < 3; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]+1) {
        if (BitCount < Params.DecodeBudget*8)  {
          bool Left = Read(&Stream.Stream);
          ++BitCount;
          i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
          if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        } else {
          InTheCut = false;
          goto GENERATE_PARTICLE_RIGHT;
        }
      }
    }
  GENERATE_PARTICLE_RIGHT:
    OutputParticles.push_back(particle_int{.Pos=BBox.Min});
    ++NParticlesDecoded;
    //RightTree = new (TreePtr++) tree;
    //RightTree->Count = 1;
  } else if (InTheCut && Mid<End) {
    split_type NextSplit = (Depth+1==Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
    if (Split == SpatialSplit)
      RightTree = DecodeTreeIntDFS(PredNode?PredNode->Right:nullptr, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    else if (Split == ResolutionSplit)
      RightTree = DecodeTreeIntDFS(LeftTree, Mid, End, GridRight, NextSplit, ResLvl+1, Depth+1);
  }

  tree* Node = nullptr;
  //if (Split == ResolutionSplit) {
  //  Node = BuildPredTree(LeftTree, RightTree, Depth, D);
  //  assert(Node && Node->Count>0);
  //} else if (Split == SpatialSplit) {
  //  if (Depth > Params.StartResolutionSplit) {
  //    Node = new (TreePtr++) tree;
  //    Node->Left  = LeftTree;
  //    Node->Right = RightTree;
  //    if (LeftTree ) Node->Count = LeftTree->Count  ; else Node->Count = 0;
  //    if (RightTree) Node->Count += RightTree->Count;
  //  }
  //}

  return Node;
}

/* we divide the particles into three trees, one for each x/y/z splits 
(so that the particles form a linear order) */
// first x, second x, third x
// 

static i64 BitsSkipped = 0;

INLINE u64
Pack3i64(const vec3i& V) { return u64(V.x & 0x1FFFFF) + (u64(V.y & 0x1FFFFF) << 21) + (u64(V.z & 0x1FFFFF) << 42); }
std::vector<particle_int>
RemoveRepeatedParticles(std::vector<particle_int>& Input) {
  std::vector<particle_int> Output;
  Output.reserve(Input.size());
  std::sort(Input.begin(), Input.end(), [](const auto& P1, const auto& P2) {
    return Pack3i64(P1.Pos) < Pack3i64(P2.Pos);
  });
  Output.push_back(Input[0]);
  for (size_t I = 1; I < Input.size(); ++I) {
    if (Input[I].Pos != Input[I-1].Pos) {
      Output.push_back(Input[I]);
    }
  }
  return Output;
}

// TODO: add the number of blocks to the .idx file
// TODO: 

//int 
//main(int Argc, cstr* Argv) {
//  f64 Accuracy = 0;
//  bool Ok = ToDouble(Argv[1], &Accuracy);
//  if (Ok)
//    TestCompressSeries(Accuracy);
//  else
//    printf("ERROR: provide an accuracy");
//  return 0;
//}

static std::vector<particle>
ReadParticles(cstr FileName) {
  if (strstr(FileName, ".xyz"))
    return ReadXYZ(FileName);
  if (strstr(FileName, ".dat"))
    return ReadCosmo(FileName);
  if (strstr(FileName, ".vtu"))
    return ReadVtu(FileName);
  if (strstr(FileName, ".pos"))
    return ReadRawParticles(FileName);
  if (strstr(FileName, ".ply"))
    return ReadPly(FileName);
  return std::vector<particle>();
}

static std::vector<particle_int>
ReadParticlesInt(cstr FileName) {
  if (strstr(FileName, ".ply"))
    return ReadPlyInt(FileName);
  if (strstr(FileName, ".vtu"))
    return ReadVtuInt(FileName);
  return std::vector<particle_int>();
}

static void
WriteParticles(cstr FileName, const std::vector<particle>& Particles) {
  if (strstr(FileName, ".xyz"))
    return WriteXYZ(FileName, Particles.begin(), Particles.end());
  if (strstr(FileName, ".ply"))
    return WritePLY(FileName, Particles.begin(), Particles.end());
  if (strstr(FileName, ".vtu"))
    return WriteVTU(FileName, Particles.begin(), Particles.end());
}

static void
WriteParticlesInt(cstr FileName, const std::vector<particle_int>& Particles) {
  if (strstr(FileName, ".ply"))
    return WritePLYInt(FileName, Particles.begin(), Particles.end());
  if (strstr(FileName, ".vtu"))
    return WriteVTU(FileName, Particles.begin(), Particles.end());
}

static i8
ComputeMaxDepth(const vec3i& Dims3) {
  i8 Depth = 0;
  i64 S = 1;
  i64 N = i64(Dims3.x) * i64(Dims3.y) * i64(Dims3.z);
  while (S < N) {
    ++Depth;
    S <<= 1;
  }
  REQUIRE(S == N);
  return Depth;
}

static vec3i
EnlargeToPow2(const vec3i& Dims3) {
  vec3i NewDims3(1, 1, 1);
  while (NewDims3.x < Dims3.x) { NewDims3.x <<= 1; }
  while (NewDims3.y < Dims3.y) { NewDims3.y <<= 1; }
  while (NewDims3.z < Dims3.z) { NewDims3.z <<= 1; }
  return NewDims3;
}

/* Process semantic3d data sets, from text to binary */
static void
ProcessSemantic3D(cstr FileNameIn, cstr FileNameOut) {
  std::vector<particle> Particles;
  Particles.reserve(1000000);
  char Line[128];
  FILE* Fp = fopen(FileNameIn, "r");
  while (fgets(Line, sizeof(Line), Fp)) {
    vec3f Pos3;
    vec3f Temp3; // discarding
    sscanf(Line, "%f %f %f %f %f %f", &Pos3.x, &Pos3.y, &Pos3.z, &Temp3.x, &Temp3.y, &Temp3.z);
    Particles.push_back(particle{.Pos = Pos3});
  }
  fclose(Fp);
  WriteParticles(FileNameOut, Particles);
}

static void
AllocateMemoryForBlocks(const grid_int& Grid) {
  grid_int G = Grid;
  for (int I = 0; I < Params.StartResolutionSplit; ++I) {
    G = SplitGrid(G, Params.DimsStr[I]-'x', SpatialSplit, side::Left);
  }
  Params.BlockDims3 = G.Dims3;
  Params.NBlocks3 = Params.Dims3 / Params.BlockDims3;
  printf("block dims = %d %d %d\n", Params.BlockDims3.x, Params.BlockDims3.y, Params.BlockDims3.z);
  Streams.resize(u64(Params.NBlocks3.x)*u64(Params.NBlocks3.y)*u64(Params.NBlocks3.z));
  printf("nblocks = %d\n", int(Streams.size()));
  OutputBlocks.resize(Streams.size());
  FOR_EACH(S, Streams) {
    InitWrite(&S->Stream, 1 << 20); // 1 MB
#if defined(FORCE_BINOMIAL)
    S->Coder.InitWrite(1 << 20);
#endif
  }
  InitWrite(&GlobalStream.Stream, 1 << 20); // 1 MB
#if defined(FORCE_BINOMIAL)
  GlobalStream.Coder.InitWrite(1 << 20);
#endif
}

static void
InitContext() {
  ContextS .resize((Params.MaxDepth+1)*Params.NLevels);
  ContextR .resize((Params.MaxDepth+1)*Params.NLevels);
  ContextTS.resize((Params.MaxDepth+1)*Params.NLevels);
  ContextTSR .resize((Params.MaxDepth+1)*Params.NLevels);
  FOR_EACH (C, ContextS  ) { C->reserve(512); }
  FOR_EACH (C, ContextR  ) { C->reserve(512); }
  FOR_EACH (C, ContextTS ) { C->reserve(512); }
  FOR_EACH (C, ContextTSR) { C->reserve(512); }
}

static std::tuple<i64, i64>
WriteBinaryFiles() {
  FOR_EACH(S, Streams) { // write the block sizes
    StreamSize += WriteBlock(FilePtr, &*S, &MetaStream);
    GrowToAccomodate(&MetaStream, 8);
    WriteVarByte(&MetaStream, S->StreamSize);
    GrowToAccomodate(&MetaStream, 8);
    WriteVarByte(&MetaStream, S->CoderSize);
  }
  { // global stream
    auto ByteCount = Size(GlobalStream.Stream);
    if (ByteCount > 0) {
      Flush(&GlobalStream.Stream);
      fwrite(GlobalStream.Stream.Stream.Data, ByteCount, 1, FilePtr);
      StreamSize += ByteCount;
    }
    GrowToAccomodate(&MetaStream, 8);
    WriteVarByte(&MetaStream, ByteCount); // global stream
  }
  { // global coder
    auto ByteCount = Size(GlobalStream.Coder.BitStream);
    if (ByteCount > 0) {
      GlobalStream.Coder.EncodeFinalize();
      fwrite(GlobalStream.Coder.BitStream.Stream.Data, ByteCount, 1, FilePtr);
      StreamSize += ByteCount;
    }
    GrowToAccomodate(&MetaStream, 8);
    WriteVarByte(&MetaStream, ByteCount); // global coder
  }
  // write the meta data
  Flush(&MetaStream);
  fwrite(MetaStream.Stream.Data, Size(MetaStream), 1, FilePtr);
  i64 MetaSize = Size(MetaStream);
  fwrite(&MetaSize, sizeof(MetaSize), 1, FilePtr);
  fclose(FilePtr);
  Dealloc(&MetaStream);
  return {StreamSize, MetaSize};
}

static void
ReadBinaryFiles(const grid_int& Grid) {
  FILE* Fp = fopen(PRINT("%s.bin", Params.InFile), "rb");
  FSEEK(Fp, 0, SEEK_END);
  i64 MetaSize = 0;
  ReadBackwardPOD(Fp, &MetaSize);
  FSEEK(Fp, -(MetaSize+(i64)sizeof(MetaSize)), SEEK_END);
  bitstream Bs;
  AllocBuf(&Bs.Stream, MetaSize);
  fread(Bs.Stream.Data, MetaSize, 1, Fp);
  InitRead(&Bs, Bs.Stream);
  grid_int G = Grid;
  for (int I = 0; I < Params.StartResolutionSplit; ++I) {
    G = SplitGrid(G, Params.DimsStr[I]-'x', SpatialSplit, side::Left);
  }
  Params.BlockDims3 = G.Dims3;
  printf("block dims = %d %d %d\n", Params.BlockDims3.x, Params.BlockDims3.y, Params.BlockDims3.z);
  Params.NBlocks3 = Grid.Dims3 / Params.BlockDims3;
  Streams.resize(u64(Params.NBlocks3.x)*u64(Params.NBlocks3.y)*u64(Params.NBlocks3.z));
  OutputBlocks.resize(Streams.size());
  FSEEK(Fp, 0, SEEK_SET);
  FOR_EACH(S, Streams) {
    { // stream
      auto ByteCount = ReadVarByte(&Bs);
      if (ByteCount > 0) {
        AllocBuf(&S->Stream.Stream, ByteCount);
        fread(S->Stream.Stream.Data, ByteCount, 1, Fp);
        InitRead(&S->Stream, S->Stream.Stream);
      }
    }
    { // coder
      auto ByteCount = ReadVarByte(&Bs);
      if (ByteCount > 0) {
        AllocBuf(&S->Coder.BitStream.Stream, ByteCount);
        fread(S->Coder.BitStream.Stream.Data, ByteCount, 1, Fp);
        S->Coder.InitRead();
      }
    }
  }
  { // global stream
    auto ByteCount = ReadVarByte(&Bs);
    if (ByteCount > 0) { // global stream
      AllocBuf(&GlobalStream.Stream.Stream, ByteCount); // 1 MB
      fread(GlobalStream.Stream.Stream.Data, ByteCount, 1, Fp);
      InitRead(&GlobalStream.Stream, GlobalStream.Stream.Stream);
    }
  }
  { // global coder
    auto ByteCount = ReadVarByte(&Bs);
    if (ByteCount > 0) { // global stream
      AllocBuf(&GlobalStream.Coder.BitStream.Stream, ByteCount); // 1 MB
      fread(GlobalStream.Coder.BitStream.Stream.Data, ByteCount, 1, Fp);
      GlobalStream.Coder.InitRead();
    }
  }
  Dealloc(&Bs);
  if (Fp) fclose(Fp);

  //FILE* Fp = fopen(PRINT("%s.bin", Params.InFile), "rb");
  //FSEEK(Fp, 0, SEEK_END);
  //i64 FirstStreamSize = 0, SecondStreamSize = 0;
  //ReadBackwardPOD(Fp, &SecondStreamSize);
  //ReadBackwardPOD(Fp, &FirstStreamSize);
  //AllocBuf(&BlockStream.Stream, FirstStreamSize);
  //AllocBuf(&Coder.BitStream.Stream, SecondStreamSize);
  //FSEEK(Fp, 0, SEEK_SET);
  //fread(BlockStream.Stream.Data, FirstStreamSize, 1, Fp);
  //if (SecondStreamSize > 0) {
  //  fread(Coder.BitStream.Stream.Data, SecondStreamSize, 1, Fp);
  //  Coder.InitRead();
  //}
  //if (Fp) fclose(Fp);
  //InitRead(&BlockStream, BlockStream.Stream);
  //i64 N = ReadVarByte(&BlockStream);
}

//ProcessSemantic3D("D:/Downloads/sg27_station8_intensity_rgb.txt", "D:/Downloads/sg27_station8_intensity_rgb.vtu");
//return 0;
//{
//  i32 Prob = 0;
//  ToInt(Argv[1], &Prob);
//  auto Particles = GenerateRandomParticles(Prob);
//  WritePLYInt(PRINT("%random-%d.ply", Prob), Particles);
//  return 0;
//}
int
main(int Argc, cstr* Argv) {
  srand(1234567);
  doctest::Context context(Argc, Argv);
  context.setAsDefaultForAssertsOutOfTestCases();
  context.setAssertHandler(Handler);
  cstr ErrorMsg = "Usage: \n"
                  "  to encode: .exe particle_file.xyz --action encode --ndims 3 --nlevels 4 --height 6 --block 2 --out output\n"
                  "  to decode: .exe compressed_file --action decode --in particle_file.idx";
  cstr Action = nullptr;
  if (!OptVal(Argc, Argv, "--action", &Action)) EXIT_ERROR(ErrorMsg);
  if (strcmp("encode", Action) == 0) Params.Action = action::Encode;
  else if (strcmp("decode", Action) == 0) Params.Action = action::Decode;
  else if (strcmp("error", Action) == 0) Params.Action = action::Error;
  else if (strcmp("convert", Action) == 0) Params.Action = action::Convert;
  else if (strcmp("dedup", Action) == 0) Params.Action = action::Dedup;
  else EXIT_ERROR(ErrorMsg);

  if (Params.Action == action::Encode) {
    if (!OptVal(Argc, Argv, "--name", &Params.OutFile)) EXIT_ERROR("missing --name");
    sprintf(Params.Name, "%s", Params.OutFile);
    if (!OptVal(Argc, Argv, "--ndims", &Params.NDims)) EXIT_ERROR("missing --ndims");
    if (!OptVal(Argc, Argv, "--nlevels", &Params.NLevels)) EXIT_ERROR("missing --nlevels");
    if (!OptVal(Argc, Argv, "--start_depth", &Params.StartResolutionSplit)) EXIT_ERROR("missing --start_depth");
    bool Budget = OptExists(Argc, Argv, "--budget");
    if (Budget) {
      OptVal(Argc, Argv, "--budget", &Params.DecodeBudget);
    }
    //Params.NoRefinement = OptExists(Argc, Argv, "--no_refinement");
    char Temp[32];
    cstr Str = Temp;
    OptVal(Argc, Argv, "--refinement", &Str);
    if (strcmp(Str, "error"     ) == 0) Params.RefinementMode = refinement_mode::ERROR_BASED;
    if (strcmp(Str, "lossless"  ) == 0) Params.RefinementMode = refinement_mode::LOSSLESS;
    if (strcmp(Str, "separation") == 0) Params.RefinementMode = refinement_mode::SEPARATION_ONLY;
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    char Buf[512]; 
    strncpy(Buf, Params.InFile, sizeof(Buf));
    CdfTable = CreateBinomialTable(BinomialCutoff);
    bool Series = OptExists(Argc, Argv, "--series");
    FILE* Tp = nullptr;
    bool Ok = false;
    i32 TimeStep = 0;
    FilePtr = fopen(PRINT("%s.bin", Params.OutFile), "wb");
    InitWrite(&MetaStream, 1<<20);
    if (!Series) 
      goto START;
    Tp = fopen(Params.InFile, "rb");
    while (Series) {
      //Ok = fgets(Buf, sizeof Buf, Tp);
      Ok = fscanf(Tp, "%s\n", Buf);
      if (TimeStep >= 2) 
        break;
START:
      ParticlesInt = ReadParticlesInt(Buf);
      if (ParticlesInt.size() == 0)
        EXIT_ERROR("No particles read");
      Params.NParticles = ParticlesInt.size();
      //if (TreePtrBackup == nullptr) {
      //  assert(PrevFramePtrBackup == nullptr);
      //  TreePtr      = new tree[Params.NParticles * 8];
      //  PrevFramePtr = new tree[Params.NParticles * 8];
      //  TreePtrBackup      = TreePtr;
      //  PrevFramePtrBackup = PrevFramePtr;
      //}
      printf("number of particles = %zu\n", ParticlesInt.size());
      double start_time = timer();
      Params.BBoxInt = ComputeBoundingBox(ParticlesInt);
      //Params.BBoxInt = bbox_int{vec3i{182, 9, 120}, vec3i{706, 1033, 376}};
      printf("bbox = (%d %d %d) - (%d %d %d)\n", 
        Params.BBoxInt.Min[0], Params.BBoxInt.Min[1], Params.BBoxInt.Min[2],
        Params.BBoxInt.Max[0], Params.BBoxInt.Max[1], Params.BBoxInt.Max[2]);
      //WriteVarByte(&BlockStream, ParticlesInt.size());
      Params.Dims3 = Params.BBoxInt.Max - Params.BBoxInt.Min + 1; //vec3i(1 << Params.LogDims3.x, 1 << Params.LogDims3.y, 1 << Params.LogDims3.z);
      printf("dims = %d %d %d\n", Params.Dims3[0], Params.Dims3[1], Params.Dims3[2]);
      Params.Dims3 = EnlargeToPow2(Params.Dims3);
      Params.BBoxInt.Max = Params.BBoxInt.Min + Params.Dims3 - 1;
      printf("enlarged dims = %d %d %d\n", Params.Dims3[0], Params.Dims3[1], Params.Dims3[2]);
      Params.LogDims3 = ComputeGrid(&ParticlesInt, MCOPY(Params.BBoxInt, .Max=Params.BBoxInt.Min+Params.Dims3), 0, ParticlesInt.size(), 0, Params.DimsStr);
      //Params.LogDims3 = ComputeGrid(&ParticlesInt, Params.BBoxInt, 0, ParticlesInt.size(), 0, Params.DimsStr);
      //yxyxyzxyzxyzxyzxyzxyzxyzxyz
      //sprintf(Params.DimsStr, "%s", "xyzxyzxyzxyzxyzxyzxyzxyzxyy");
                                       //xyzxyzyyyzyzyzyzyzxyzxyzxyz
      OptVal(Argc, Argv, "--w3", &Params.AddToW3);
      Params.W3[0] = Params.Dims3[0]/(1<<(Params.LogDims3[0]));
      Params.W3[1] = Params.Dims3[1]/(1<<(Params.LogDims3[1]));
      Params.W3[2] = Params.Dims3[2]/(1<<(Params.LogDims3[2]));
      auto L = Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z;
      for (int I = 0; I < Params.AddToW3; ++I) {
        int D = Params.DimsStr[L-I-1] - 'x';
        Params.W3[D] <<= 1;
      }
      printf("log dims = %d %d %d\n", Params.LogDims3[0], Params.LogDims3[1], Params.LogDims3[2]);
      printf("w3 = %d %d %d\n", Params.W3[0], Params.W3[1], Params.W3[2]);
      Params.Dims3 = Params.Dims3 / Params.W3;
      Params.MaxDepth = ComputeMaxDepth(Params.Dims3);
      //InitContext(); // uncomment to use contexts
      printf("max depth = %d\n", Params.MaxDepth);
      grid_int Grid{.From3 = vec3i(0), .Dims3 = Params.Dims3, .Stride3 = vec3i(1)};
      AllocateMemoryForBlocks(Grid);
      printf("bounding box = (" PRIvec3i ") - (" PRIvec3i ")\n", EXPvec3(Params.BBoxInt.Min), EXPvec3(Params.BBoxInt.Max));
      printf("dims string = %s\n", Params.DimsStr);
      i64 N = ParticlesInt.size();
      i8 T = Msb(u64(N)) + 1;
      split_type Split = (Params.NLevels>1 && Params.StartResolutionSplit==0) ? ResolutionSplit : SpatialSplit;
      printf("--------------- Encoding %s\n", Buf);
      tree* MyNode = nullptr;
      //MyNode = BuildTreeIntPredict(TimeStep==0?nullptr:PrevFramePtr, ParticlesInt, 0, ParticlesInt.size(), T, Grid, Split, 0, 0);
      q_item_int Q {
        .Begin = 0,
        .End = N,
        .Grid = Grid,
        .ResLvl = 0,
        .Depth = 0,
        .Split = Split,
      };
      if (OptExists(Argc, Argv, "--adaptive")) {
        BuildIntAdaptive(ParticlesInt, Q);
      } else if (OptExists(Argc, Argv, "--bfs")) {
        BuildTreeIntBFS(Q, ParticlesInt);
      } else {
        BuildTreeIntDFS(ParticlesInt, 0, N, Grid, Split, 0, 0);
      }
      PrevFramePtr = MyNode;
      //TreePtr = MyNode;
      //PrevFramePtr = PrevFramePtrBackup;
      //std::swap(TreePtr, PrevFramePtr);
      //std::swap(TreePtrBackup, PrevFramePtrBackup);
      // TODO: free the previous frame's memory
      double dec_time = timer() - start_time;
      printf("Time: %f s\n", dec_time);
      ++TimeStep;
    }
    //delete[] TreePtrBackup;
    //delete[] PrevFramePtrBackup;
    auto [StreamSize, MetaSize] = WriteBinaryFiles();
    WriteMetaFile(Params, PRINT("%s.idx", Params.OutFile));
    printf("%s\n", Params.DimsStr);
    //printf("Uniform code size 1                = %lld\n", (UniformCodeSize1 + 7) / 8);
    printf("Max depth                          = %d\n", Params.MaxDepth);
    printf("Binomial stream size               = %lld\n", (BinomialCodeSize + 7) / 8);
    //printf("Uniform code size 2                = %lld\n", (UniformCodeSize2 + 7) / 8);
    printf("Range code size                    = %f\n",   (RangeCodeSize + 7) / 8);
    printf("Non-predicted code size            = %lld\n", (NonPredictedCodeSize + 7) / 8);
    printf("predicted node count               = %lld\n", PredictedNodeCount);
    printf("non predicted node count           = %lld\n", NonPredictedNodeCount);
    printf("Stream size                        = %lld\n", StreamSize);
    printf("Refinement code size               = %lld\n", (RefinementCodeLength + 7 ) / 8);
    printf("RMSE = %f\n", sqrt(RMSE / (ParticlesInt.size() * Params.NDims)));
    printf("# particles = %lld\n", NParticlesDecoded);
    printf("Average ratio = %f Ratio count = %lld\n", Ratio / RatioCount, RatioCount);
    printf("Nodes with more empty cells count = %lld\n", NodesWithMoreEmptyCellsCount);
    printf("Nodes with more particles count = %lld\n", NodesWithMoreParticlesCount);
    ///* dump the debug info */
    //FILE* Ff = fopen("debug.dat", "wb");
    //i64 DebugSize = SRList.size();
    //fwrite(&DebugSize, sizeof(DebugSize), 1, Fp);
    //for (i64 I = 0; I < DebugSize; ++I) {
    //  fwrite(&SRList[I], sizeof(SRList[I]), 1, Fp);
    //}
    //fclose(Ff);
  /* ---------------- DECODING ------------------*/
  } else if (Params.Action == action::Decode) { /* decoding */
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    ReadMetaFile(PRINT("%s.idx", Params.InFile));
    OptVal(Argc, Argv, "--sub_ratio", &Params.SubsamplingRatio);
    OptVal(Argc, Argv, "--decode_depth", &Params.DecodeDepth);
    bool Budget = OptExists(Argc, Argv, "--budget");
    if (Budget) {
      OptVal(Argc, Argv, "--budget", &Params.DecodeBudget);
    }
    i64 N = Params.NParticles;
    printf("%s\n", Params.DimsStr);
    Params.MaxDepth = ComputeMaxDepth(Params.Dims3);
    //InitContext();
    printf("baseheight = %d maxheight = %d\n", Params.BaseHeight, Params.MaxHeight);
    double start_time = timer();
    uint64_t dec_start_time = __rdtsc();
    //BinomialTables = CreateGeneralBinomialTables();
    CdfTable = CreateBinomialTable(BinomialCutoff);
    printf("DecodeAccuracy = %f\n", Params.DecodeAccuracy);
    grid_int Grid{.From3 = vec3i(0), .Dims3 = Params.Dims3, .Stride3 = vec3i(1)};
    printf("bounding box = (" PRIvec3i ") - (" PRIvec3i ")\n", EXPvec3(Params.BBoxInt.Min), EXPvec3(Params.BBoxInt.Max));
    ReadBinaryFiles(Grid);
    split_type Split = SpatialSplit;
    if (Params.NLevels>1 && Params.StartResolutionSplit==0)
      Split = ResolutionSplit;
    /* read the debug info */
    //FILE* Ff = fopen("debug.dat", "rb");
    //i64 DebugSize = 0;
    //fread(&DebugSize, sizeof(DebugSize), 1, Ff);
    //SRList.resize(DebugSize);
    //for (i64 I = 0; I < DebugSize; ++I) {
    //  fread(&SRList[I], sizeof(SRList[I]), 1, Ff);
    //}
    //fclose(Ff);
    //Treeptr = new tree[Params.NParticles * 10]; // TODO: avoid this
    auto TreePtrBackup = TreePtr;
    ParticlesInt.reserve(N);
    tree* MyNode = nullptr;
    //MyNode = DecodeTreeIntPredict(nullptr, ParticlesInt, 0, N, Msb(u64(N))+1, Grid, Split, 0, 0);
    q_item_int Q {
      .Begin = 0,
      .End = N,
      .Grid = Grid,
      .ResLvl = 0,
      .Depth = 0,
      .Split = Split,
    };
    if (OptExists(Argc, Argv, "--adaptive")) {
      DecodeIntAdaptive(Q);
    } else if (OptExists(Argc, Argv, "--bfs")) {
      DecodeTreeIntBFS(Q);
    } else {
      DecodeTreeIntDFS(nullptr, 0, N, Grid, Split, 0, 0);
    }
    //delete[] TreePtrBackup;
    uint64_t dec_clocks = __rdtsc() - dec_start_time;
    double dec_time = timer() - start_time;
    printf("%lld clocks, %f s\n", dec_clocks, dec_time);
    //WritePLYInt(PRINT("%s.ply", Params.OutFile), OutputParticles.begin(), OutputParticles.end());
    printf("num particles decoded = %lld\n", NParticlesDecoded);
    printf("bit count = %d bytes\n", BitCount/8);
    vec3f Scale3 = 30.0 / vec3f(Params.BBoxInt.Max-Params.BBoxInt.Min);
    if (Budget) {
      WriteXYZ(PRINT("%s.xyz", Params.OutFile), OutputParticles.begin(), OutputParticles.end(), Params.BBoxInt.Min, Scale3);
    }
  //================= ERROR =========================
  } else if (Params.Action == action::Error) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    if (!OptVal(Argc, Argv, "--dims", &Params.Dims3)) EXIT_ERROR("missing --dims");
    auto Particles1 = ReadParticlesInt(Params.InFile);
    auto Particles2 = ReadParticlesInt(Params.OutFile);
    f32 Err1 = Error3(Particles1, Particles2, Params.Dims3);
    //f32 Err2 = Error3(Particles2, Particles1, Params.Dims3);
    //printf("error = %f %f %f\n", Err1, Err2, MAX(Err1, Err2));
    printf("error = %f\n", Err1);
    //if (!CheckSame(Particles1, Particles2))
    //  printf("not same\n");
    //else
    //  printf("same\n");
  //================= CONVERT =======================
  } else if (Params.Action == action::Convert) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    bool Quantize = OptExists(Argc, Argv, "--quantize");
    f32 MaxAbsX = 0, MaxAbsY = 0, MaxAbsZ = 0;
    //Particles = ReadParticles(Params.InFile);
    ParticlesInt = ReadParticlesInt(Params.InFile);
    fprintf(stderr, "Done reading particles\n");
    //ParticlesInt.resize(Particles.size());
    if (Quantize) { // quantize everything to 23 bits
      FOR_EACH(P, Particles) {
        MaxAbsX = MAX(MaxAbsX, fabs(P->Pos.x));
        MaxAbsY = MAX(MaxAbsY, fabs(P->Pos.y));
        MaxAbsZ = MAX(MaxAbsZ, fabs(P->Pos.z));
      }
      int EMaxX = Exponent(MaxAbsX), EMaxY = Exponent(MaxAbsY), EMaxZ = Exponent(MaxAbsZ);
      /* quantize */
      int Bits = 21;
      f64 ScaleX = ldexp(1, Bits-1-EMaxX), ScaleY = ldexp(1, Bits-1-EMaxY), ScaleZ = ldexp(1, Bits-1-EMaxZ);
      for (size_t I = 0; I < Particles.size() ; ++I) {
        ParticlesInt[I].Pos.x = i32(ScaleX * Particles[I].Pos.x);
        ParticlesInt[I].Pos.y = i32(ScaleY * Particles[I].Pos.y);
        ParticlesInt[I].Pos.z = i32(ScaleZ * Particles[I].Pos.z);
      }
      fprintf(stderr, "Done quantizing\n");
      ParticlesInt = RemoveRepeatedParticles(ParticlesInt);
      fprintf(stderr, "Writing particles\n");
      WriteParticlesInt(Params.OutFile, ParticlesInt);
    } else if (OptExists(Argc, Argv, "--ospray")) {
      //Params.BBox = ComputeBoundingBox(Particles);
      Params.BBoxInt = ComputeBoundingBox(ParticlesInt);
      vec3f Scale3 = 30.0 / vec3f(Params.BBoxInt.Max - Params.BBoxInt.Min);
      WriteXYZ(PRINT("%s.xyz", Params.OutFile), ParticlesInt.begin(), ParticlesInt.end(), Params.BBoxInt.Min, Scale3);      
      //WriteVTU(PRINT("%s.vtu", Params.OutFile), Particles.begin(), Particles.end(), Params.BBox.Min, Scale3);
    } else {
      fprintf(stderr, "Writing particles\n");
      WriteParticles(Params.OutFile, Particles);
    }
  } else if (Params.Action == action::Dedup) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    auto ParticlesInt = ReadParticlesInt(Params.InFile);
    ParticlesInt = RemoveRepeatedParticles(ParticlesInt);
    WriteParticlesInt(Params.OutFile, ParticlesInt);
  }

  //RandomLevels(&Particles);
}
// TODO: we need to swap the roles of Particles1 and Particles2 when computing the error
// and also pass in the bounding box from outside the function Error

