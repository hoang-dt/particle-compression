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

static i64 BlockBytesRead = 0;

static bool
ReadBlock(i8 Level, u64 BlockId, u8 Height) {
  REQUIRE(Level < Params.NLevels);
//  printf("--------- reading level %d block %llu height %d\n", Level, BlockId, Height);

  FILE* Fp = nullptr;
  /* read the block offsets if not done so */
  if (BlockOffsets[Level].empty()) {
    // read the block bytes
    Fp = fopen(PRINT("%s-%d.bin", Params.Name, Level), "rb");
    if (!Fp)
      return false;
    FSEEK(Fp, 0, SEEK_END);
    ReadBackwardPOD(Fp, &MaxBlockSize);
    u64 NBlocks = 0;
    ReadBackwardPOD(Fp, &NBlocks);
    REQUIRE(BlockId < NBlocks);
    BlockOffsets[Level].resize(NBlocks);
    buffer Buf((byte*)BlockOffsets[Level].data(), (i64)sizeof(block_meta) * NBlocks);
    ReadBackwardBuffer(Fp, &Buf);
    BlockBytes[Level] = BlockOffsets[Level];
    u64 S = 0;
    FOR(u64, I, 0, NBlocks) {
      u64 Temp = BlockOffsets[Level][I].Size;
      BlockOffsets[Level][I].Size = S;
      S += Temp;
    }
    std::sort(BlockOffsets[Level].begin(), BlockOffsets[Level].end());
    std::sort(BlockBytes[Level].begin(), BlockBytes[Level].end());
  }
  auto It = std::lower_bound(BlockOffsets[Level].begin(), BlockOffsets[Level].end(), block_meta{.Size = 0, .BlockId = BlockId});
  if (It == BlockOffsets[Level].end() || It->BlockId != BlockId) {
    printf("    NOT FOUND !!!!\n");
    return false;
  }

  if (!Fp) 
    Fp = fopen(PRINT("%s-%d.bin", Params.Name, Level), "rb");
  FSEEK(Fp, It->Size, SEEK_SET);
  bitstream& Bs = (Height <= Params.BaseHeight) ? BlockStreams[Level] : RefBlockStreams[Height - Params.BaseHeight - 1];
  Rewind(&Bs);
  GrowToAccomodate(&Bs, MaxBlockSize);
  It = std::lower_bound(BlockBytes[Level].begin(), BlockBytes[Level].end(), block_meta{.Size = 0, .BlockId = BlockId});
  BlockBytesRead += It->Size;
  fread(Bs.Stream.Data, MaxBlockSize, 1, Fp);
  fclose(Fp);

  return true;
}

const static long all_nbins = 1 << 30;
const static double epsilon = 1.0 / double(all_nbins);

INLINE static i64
DecodeNode(bitstream* Bs, i64 M) {
  // TODO: use binomial coding
//  return ReadVarByte(Bs);
  return (i64)DecodeCenteredMinimal((u32)M + 1, Bs);
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
    Block->NParticles += (int)M;
//    printf("%lld %lld\n", Block->Nodes[I], Block->Nodes[I - 1]);
    assert(RES_PARENT(I) == RES_PARENT(I - 1));
  }
}

// TODO: and from tree depth to bounding box
static void
DecodeRefBlock(bitstream* Bs, i8 Level, u64 BlockIdx, block_table* AllBlocks) {
  //InitRead(Bs, Bs->Stream);
  REQUIRE(AllBlocks->size() > Level);
  auto& Blocks = (*AllBlocks)[Level];
  if (Blocks.size() <= BlockIdx) {
    Blocks.resize(BlockIdx * 3 / 2 + 1);
  }
  Blocks[BlockIdx] = block(Bs);
//  const block& CurrBlock = Blocks[BlockIdx];
//  i64 NumBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(Level);
//  u64 ParentBlockIdx = BlockIdx - NumBlocksAtLeaf;
//  const auto& ParentBlock = (*AllBlocks)[Level][ParentBlockIdx];
//  REQUIRE(ParentBlock.NParticles <= CurrBlock.BitSet.NBits);
//  int NNodes = ParentBlock.NumNodes();
//  InitRead(Bs, Bs->Stream);
//  FOR(int, I, 0, NNodes) {
//    REQUIRE(ParentBlock.Get(I) <= 1);
//    if (ParentBlock.Get(I) == 1) {
//      bool Bit = Read(Bs);
//      printf("  refinement bit %d\n", Bit);
//    }
//  }
//  ParentBlock.Nodes
  //printf("%lld %lld\n", Block.Get(I), Block.Get(I+1));
}

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
    Block.Nodes[1] = ResBlock.Nodes[LEVEL_TO_NODE(Level)];
  }
  for (u64 K = FirstNodeIdx; K < LastNodeIdx; K += 2) {
    u64 I = NODE_INDEX_IN_BLOCK(K);
    u64 J = K / 2; // (global) parent index
    i64 M = Blocks[NODE_TO_BLOCK_INDEX(J)].Nodes[NODE_INDEX_IN_BLOCK(J)];
    if (M > 0) {
      Block.Nodes[I    ] = DecodeNode(Bs, M); // left child
      Block.Nodes[I + 1] = M - Block.Nodes[I]; // right child
      assert(Block.Nodes[I] >= 0 && Block.Nodes[I] <= Params.NParticles);
      assert(Block.Nodes[I + 1] >= 0 && Block.Nodes[I + 1] <= Params.NParticles);
//      printf("%lld %lld\n", Block.Nodes[I], Block.Nodes[I+1]);
    }
  }
}


struct block_data {
  i8 Level = 0;
  u8 Height = 0;
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
  u64 BlockId = 0;
  f32 Error = 0;
};
INLINE bool operator<(const block_priority& Lhs, const block_priority& Rhs) {
  bool LvlLess   = Lhs.Level > Rhs.Level;
  bool LvlEq     = Lhs.Level == Rhs.Level;
  bool BlockLess = Lhs.BlockId > Rhs.BlockId;
  bool BlockEq   = Lhs.BlockId == Rhs.BlockId;
  bool ErrorLess = Lhs.Error < Rhs.Error;
  bool ErrorEq   = Lhs.Error == Rhs.Error;
  return ErrorLess || (ErrorEq && (LvlLess || (LvlEq && (BlockLess || BlockEq))));
}

static DynamicHeap<block_data, block_priority> Heap;

INLINE double
NodeVolume(i8 Level, i64 NodeIdx) {
  vec3f V3 = Params.BBox.Max - Params.BBox.Min;
  float V = V3.x * V3.y * V3.z;
  int H = LOG2_FLOOR(NodeIdx) + LEVEL_TO_HEIGHT(Level);
  double S = ldexp(V, -H);
  return S;
}

INLINE float
NodeVolume(int Height) {
  vec3f V3 = Params.BBox.Max - Params.BBox.Min;
  float V = V3.x * V3.y * V3.z;
  double S = ldexp(V, -Height);
  return (float)S;
}

struct tree_node {
  i8 Level = 0;
  u8 Height = 0;
  u64 NodeId = 0;
  grid Grid;
  i8 D = 0;
};
INLINE bool operator<(const tree_node& Lhs,  const tree_node& Rhs) {
  bool LvlLess   = Lhs.Level > Rhs.Level;
  bool LvlEq     = Lhs.Level == Rhs.Level;
  bool NodeLess = Lhs.NodeId > Rhs.NodeId;
  return LvlLess || (LvlEq && NodeLess);
}

struct tree_node_priority {
  i8 Level = 0;
  u64 NodeId = 0;
  f32 Error = 0;
};
INLINE bool operator<(const tree_node_priority& Lhs, const tree_node_priority& Rhs) {
  bool LvlLess   = Lhs.Level > Rhs.Level;
  bool LvlEq     = Lhs.Level == Rhs.Level;
  bool NodeLess = Lhs.NodeId > Rhs.NodeId;
  bool NodeEq   = Lhs.NodeId == Rhs.NodeId;
  bool ErrorLess = Lhs.Error < Rhs.Error;
  bool ErrorEq   = Lhs.Error == Rhs.Error;
  return ErrorLess || (ErrorEq && (LvlLess || (LvlEq && (NodeLess || NodeEq))));
}

static DynamicHeap<tree_node, tree_node_priority> NodeHeap;

// TODO: figure out what the code below does
//static void
//RefineNodes() {
//  tree_node          TopNode;
//  tree_node_priority NodePriority;
//  bool BlockExists = false;
//
////  REQUIRE(LvlBlocks[TopBlock.Level].size() > TopBlock.BlockId);
//  block_data LeftChild, RightChild;
//  float LeftError = 0, RightError = 0;
//  float LeftVol = 0, RightVol = 0;
//  i64 LeftN = 0, RightN = 0;
//  if (TopNode.Level == Params.NLevels) { // resolution block
//    auto& Nodes = Blocks[TopNode.Level][0].Nodes;
//    int NNodes = Params.NLevels * 2 - 1;
//    REQUIRE(NNodes == Nodes.size());
//    if (Nodes[TopNode.NodeId] == 0) return;
//    if (TopNode.NodeId + 1 == NNodes || !IS_EVEN(TopNode.NodeId)) {
//      i8 Level = (2 * (Params.NLevels - 1) - (NodeIdx - 1)) / 2;
//      // NOTE: we have only one child instead of two (BlockBits >= 1)
//      LeftChild = block_data{
//        .Level = Level,
//        .Height = u8(LEVEL_TO_HEIGHT(Level)),
//        .BlockId = 0
//      };
//      if (LeftChild.Height <= Params.MaxHeight) {
//        LeftError = NodeVolume(LEVEL_TO_HEIGHT(Level)) / Nodes[NodeIdx];
//        Heap.insert(LeftChild, block_priority{.Level = Level, .BlockId = 0, .Error = LeftError});
//      }
//    }
//  } else if (TopBlock.Height < Params.BaseHeight) { // just a regular block on some resolution
//    LeftChild  = block_data{
//      .Level   = TopBlock.Level,
//      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
//      .BlockId = TopBlock.BlockId * 2 // NOTE: if BlockId == 0, the true (only) child is the right child
//    };
//    RightChild = block_data{
//      .Level   = TopBlock.Level,
//      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
//      .BlockId = TopBlock.BlockId * 2 + 1
//    };
//    int H = TopBlock.BlockId == 0 ? TopBlock.Height + Params.BlockBits - 1 : TopBlock.Height;
//    float Vol = NodeVolume(H);
//    FOR(int, NodeIdx, 0, (int)Nodes.size()) {
//      if (Nodes[NodeIdx] == 0) continue;
//      u64 GlobalNodeIdx = TopBlock.BlockId * POW2(Params.BlockBits) + NodeIdx;
//      u64 ChildrenBlockIdx = NODE_TO_BLOCK_INDEX(GlobalNodeIdx * 2);
//      if (ChildrenBlockIdx == TopBlock.BlockId) continue;
//      assert(ChildrenBlockIdx == LeftChild.BlockId || ChildrenBlockIdx == RightChild.BlockId);
//      if (ChildrenBlockIdx == LeftChild.BlockId) {
//        LeftN += Nodes[NodeIdx];
//        LeftVol += Vol;
//      } else {
//        RightN += Nodes[NodeIdx];
//        RightVol += Vol;
//      }
//    }
//    LeftError = LeftVol / LeftN;
//    RightError = RightVol / RightN;
//    if (LeftChild.Level <= Params.MaxLevel && TopBlock.BlockId != 0 && LeftError > 0 && LeftChild.Height <= Params.MaxHeight)
//      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
//    if (RightChild.Level <= Params.MaxLevel && RightError > 0 && RightChild.Height <= Params.MaxHeight)
//      Heap.insert(RightChild, block_priority{.Level = RightChild.Level, .BlockId = RightChild.BlockId, .Error = RightError});
//  } else if (TopBlock.Height < Params.MaxHeight) { // refinement level
//    i64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(TopBlock.Level);
//    LeftChild  = block_data {
//      .Level   = TopBlock.Level,
//      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
//      .BlockId = TopBlock.BlockId + NBlocksAtLeaf
//    };
//    if (LeftChild.Level <= Params.MaxLevel && LeftChild.Height <= Params.MaxHeight) {
//      LeftError = TopPriority.Error * 0.5f;
//      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
//    }
//  }
//  return true;
//}


/* Read the next most important block and add its two children (if existed) to the heap
 * Return false if there is no more block to load */
static bool
RefineByError() {
  block_data     TopBlock;
  block_priority TopPriority;
  bool BlockExists = false;
  while (!BlockExists) {
    if (Heap.empty()) break;
    Heap.top(TopBlock, TopPriority);
    Heap.pop();
    if (TopBlock.Level == Params.NLevels)
      BlockExists = ReadResBlock();
    else
      BlockExists = ReadBlock(TopBlock.Level, TopBlock.BlockId, TopBlock.Height);
  }
  if (!BlockExists)
    return false;

  if (TopBlock.Level == Params.NLevels)
    DecodeResBlock(&BlockStreams[TopBlock.Level], &Blocks[TopBlock.Level][0]);
  else if (TopBlock.Height <= Params.BaseHeight)
    DecodeBlock(&BlockStreams[TopBlock.Level], TopBlock.Level, TopBlock.BlockId, &Blocks);
  else
    DecodeRefBlock(&RefBlockStreams[TopBlock.Height - Params.BaseHeight - 1], TopBlock.Level, TopBlock.BlockId, &Blocks);
//  REQUIRE(LvlBlocks[TopBlock.Level].size() > TopBlock.BlockId);
  block_data LeftChild, RightChild;
  float LeftError = 0, RightError = 0;
  float LeftVol = 0, RightVol = 0;
  i64 LeftN = 0, RightN = 0;
  auto& Nodes = Blocks[TopBlock.Level][TopBlock.BlockId].Nodes;
  if (TopBlock.Level == Params.NLevels) { // resolution block
    int NNodes = Params.NLevels * 2 - 1;
    REQUIRE(NNodes == Nodes.size());
    FOR(int, NodeIdx, 0, NNodes) {
      if (Nodes[NodeIdx] == 0) continue;
      if (NodeIdx + 1 != NNodes && IS_EVEN(NodeIdx))
        continue;
      i8 Level = (2 * (Params.NLevels - 1) - (NodeIdx - 1)) / 2;
      // NOTE: we have only one child instead of two (BlockBits >= 1)
      LeftChild = block_data{
        .Level = Level,
        .Height = u8(LEVEL_TO_HEIGHT(Level)),
        .BlockId = 0
      };
      if (LeftChild.Height <= Params.MaxHeight) {
        LeftError = NodeVolume(LEVEL_TO_HEIGHT(Level)) / Nodes[NodeIdx];
        Heap.insert(LeftChild, block_priority{.Level = Level, .BlockId = 0, .Error = LeftError});
      }
    }
  } else if (TopBlock.Height < Params.BaseHeight) { // just a regular block on some resolution
    LeftChild  = block_data{
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId * 2 // NOTE: if BlockId == 0, the true (only) child is the right child
    };
    RightChild = block_data{
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId * 2 + 1
    };
    int H = TopBlock.BlockId == 0 ? TopBlock.Height + Params.BlockBits - 1 : TopBlock.Height;
    float Vol = NodeVolume(H);
    FOR(int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      u64 GlobalNodeIdx = TopBlock.BlockId * POW2(Params.BlockBits) + NodeIdx;
      u64 ChildrenBlockIdx = NODE_TO_BLOCK_INDEX(GlobalNodeIdx * 2);
      if (ChildrenBlockIdx == TopBlock.BlockId) continue;
      assert(ChildrenBlockIdx == LeftChild.BlockId || ChildrenBlockIdx == RightChild.BlockId);
      if (ChildrenBlockIdx == LeftChild.BlockId) {
        LeftN += Nodes[NodeIdx];
        LeftVol += Vol;
      } else {
        RightN += Nodes[NodeIdx];
        RightVol += Vol;
      }
    }
    LeftError = LeftVol / LeftN;
    RightError = RightVol / RightN;
    if (LeftChild.Level <= Params.MaxLevel && TopBlock.BlockId != 0 && LeftError > 0 && LeftChild.Height <= Params.MaxHeight)
      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
    if (RightChild.Level <= Params.MaxLevel && RightError > 0 && RightChild.Height <= Params.MaxHeight)
      Heap.insert(RightChild, block_priority{.Level = RightChild.Level, .BlockId = RightChild.BlockId, .Error = RightError});
  } else if (TopBlock.Height < Params.MaxHeight) { // refinement level
    i64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(TopBlock.Level);
    LeftChild  = block_data {
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId + NBlocksAtLeaf
    };
    if (LeftChild.Level <= Params.MaxLevel && LeftChild.Height <= Params.MaxHeight) {
      LeftError = TopPriority.Error * 0.5f;
      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
    }
  }
  return true;
}

static void
RefineLeftToRight() {
  // TODO: just refine the tree from left to right, no priority
}

static bool
RefineByLevel() {
  block_data     TopBlock;
  block_priority TopPriority;
  bool BlockExists = false;
  while (!BlockExists) {
    if (Heap.empty()) break;
    Heap.top(TopBlock, TopPriority);
    Heap.pop();
    if (TopBlock.Level == Params.NLevels)
      BlockExists = ReadResBlock();
    else
      BlockExists = ReadBlock(TopBlock.Level, TopBlock.BlockId, TopBlock.Height);
  }
  if (!BlockExists)
    return false;

//  printf("level %d block %llu\n", TopBlock.Level, TopBlock.BlockId);
  if (TopBlock.Level == Params.NLevels)
    DecodeResBlock(&BlockStreams[TopBlock.Level], &Blocks[TopBlock.Level][0]);
  else if (TopBlock.Height <= Params.BaseHeight)
    DecodeBlock(&BlockStreams[TopBlock.Level], TopBlock.Level, TopBlock.BlockId, &Blocks);
  else // refinement block
    DecodeRefBlock(&RefBlockStreams[TopBlock.Height - Params.BaseHeight - 1], TopBlock.Level, TopBlock.BlockId, &Blocks);
//  REQUIRE(LvlBlocks[TopBlock.Level].size() > TopBlock.BlockId);

  /* enqueue children blocks */
  block_data LeftChild, RightChild;
  float LeftError = 0, RightError = 0;
  auto& Nodes = Blocks[TopBlock.Level][TopBlock.BlockId].Nodes;
  if (TopBlock.Level == Params.NLevels) { // resolution block, one child for each non-even node
    int NNodes = Params.NLevels * 2 - 1;
    REQUIRE(NNodes == Nodes.size());
    FOR(int, NodeIdx, 0, NNodes) {
      if (Nodes[NodeIdx] == 0) continue;
      if (NodeIdx + 1 != NNodes && IS_EVEN(NodeIdx))
        continue;
      i8 Level = (2 * (Params.NLevels - 1) - (NodeIdx - 1)) / 2;
      // NOTE: we have only one child instead of two (BlockBits >= 1)
      LeftChild  = block_data{
        .Level   = Level,
        .Height  = u8(LEVEL_TO_HEIGHT(Level)), // NOTE: assume that the child block implicitly contains the level-root node
        .BlockId = 0
      };
      if (LeftChild.Height <= Params.MaxHeight) {
        LeftError = 1;
        Heap.insert(LeftChild, block_priority{.Level = Level, .BlockId = 0, .Error = LeftError});
      }
    }
  } else if (TopBlock.Height < Params.BaseHeight) { // regular block, each having 2 children
    LeftChild  = block_data{
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId * 2 // NOTE: if BlockId == 0, the true (only) child is the right child
    };
    RightChild = block_data{
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId * 2 + 1
    };
    FOR(int, NodeIdx, 0, (int)Nodes.size()) {
      if (Nodes[NodeIdx] == 0) continue;
      u64 GlobalNodeIdx = TopBlock.BlockId * POW2(Params.BlockBits) + NodeIdx;
      u64 ChildrenBlockIdx = NODE_TO_BLOCK_INDEX(GlobalNodeIdx * 2);
      if (ChildrenBlockIdx == TopBlock.BlockId) continue; // NOTE: to avoid parent block 0 and child block 0
      assert(ChildrenBlockIdx == LeftChild.BlockId || ChildrenBlockIdx == RightChild.BlockId);
      if (ChildrenBlockIdx == LeftChild.BlockId)
        LeftError = 1;
      else
        RightError = 1;
    }
    if (TopBlock.BlockId != 0 && LeftError > 0 && LeftChild.Height <= Params.MaxHeight)
      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
    if (RightError > 0 && RightChild.Height <= Params.MaxHeight)
      Heap.insert(RightChild, block_priority{.Level = RightChild.Level, .BlockId = RightChild.BlockId, .Error = RightError});
  } else if (TopBlock.Height < Params.MaxHeight) { // refinement level, each block has only one child
    i64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(TopBlock.Level);
    LeftChild  = block_data {
      .Level   = TopBlock.Level,
      .Height  = TopBlock.BlockId == 0 ? u8(TopBlock.Height + Params.BlockBits) : u8(TopBlock.Height + 1),
      .BlockId = TopBlock.BlockId + NBlocksAtLeaf
    };
    if (LeftChild.Height <= Params.MaxHeight) {
      LeftError = 1;
      Heap.insert(LeftChild, block_priority{.Level = LeftChild.Level, .BlockId = LeftChild.BlockId, .Error = LeftError});
    }
  }

  return true;
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

static i64 NCount2 = 0;

/* Generate N random particles inside Grid */
static void
GenerateParticlesPerNode(i64 N, const grid& Grid, const vec3f& W3) {
  if (N == 0) return;
  //if (N <= Params.MaxParticleSubSampling)
  //  N = 1;
  //printf("generating %lld particles\n", N);
  assert(Grid.Dims3.x >= 1 && Grid.Dims3.y >= 1 && Grid.Dims3.z >= 1);
  //GridPoints.reserve(N);
  //GridPoints.clear();
  static std::vector<vec3f> GridPoints; // stores the grid points that contain the (to be generated) particles
  GridPoints.resize(N);
  //vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
  vec3f Dims3 = Grid.Dims3;
  
  i64 NElems = N;
  i64 I = 0;
//  f32 M = f32(Dims3.z) * f32(Dims3.y) * f32(Dims3.x);
  FOR(float, Z, 0, Dims3.z) {
  FOR(float, Y, 0, Dims3.y) {
  FOR(float, X, 0, Dims3.x) {
    if (I < N) {
      GridPoints[I] = Grid.From3 + Grid.Stride3 * vec3f(X, Y, Z);
    } else {
      ++NElems;
      //f32 K = rand() * 1.0 / RAND_MAX;
      i64 J = rand() % NElems; // exclusive
      if (J < N)
      //if (K < N * 1.0 / M && GridPoints.size() < N)
        GridPoints[J] = Grid.From3 + Grid.Stride3 * vec3f(X, Y, Z);
        //GridPoints.push_back(Grid.From3 + Grid.Stride3 * vec3f(X, Y, Z));
    }
    ++I;
  }}}
  FOR_EACH(P, GridPoints) {
    bbox BBox{
      .Min = Params.BBox.Min + (vec3f(*P)) * W3,
      .Max = Params.BBox.Min + (vec3f(*P) + 1) * W3
    };
    GenerateOneParticle(BBox);
  }
  NCount2 += GridPoints.size();
}

//std::set<u64> Nodes;

INLINE static bool
GetNode(const tree_node& Node, i64* N) {
  //printf("getting node %lld\n", Node.NodeId);
  //if (Nodes.find(Node.NodeId) == Nodes.end())
  //  Nodes.insert(Node.NodeId);
  //else
  //  printf("---------------something is wrong!!!!!!\n");
  REQUIRE(Node.Height <= Params.BaseHeight);
  u64 BlockId = NODE_TO_BLOCK_INDEX(Node.NodeId);
  if (Blocks.size() <= Node.Level || Blocks[Node.Level].size() <= BlockId || Blocks[Node.Level][BlockId].Nodes.empty())
    return false;
  const block& Block = Blocks[Node.Level][BlockId];
  *N = Block.Nodes[NODE_INDEX_IN_BLOCK(Node.NodeId)];
  return true;
}

INLINE static bool
GetRefNode(const tree_node& Node, u8* Bit) {
  REQUIRE(Node.Height > Params.BaseHeight);
  u64 BlockId = NODE_TO_BLOCK_INDEX(Node.NodeId);
  if (Blocks.size() <= Node.Level || Blocks[Node.Level].size() <= BlockId || Size(Blocks[Node.Level][BlockId].Bs.Stream) == 0)
    return false;
  block& Block = Blocks[Node.Level][BlockId];
  *Bit = (u8)Read(&Block.Bs);
  return true;
}

i64 NCount = 0;

static i64
GenerateParticles(const tree_node& Node) {
  REQUIRE(Node.NodeId != 0);
  assert(Node.Grid.Dims3.x >= 1 && Node.Grid.Dims3.y >= 1 && Node.Grid.Dims3.z >= 1);
  i64 N = 0;
//  if (Node.Height < Params.BaseHeight) { // regular node, 2 children
//    if (GetNode(Node, &N) && N > 0) {
//      REQUIRE(N <= Params.NParticles);
//      if (N <= Params.MaxParticleSubSampling) { // return 1 particle for all N particles
////        GenerateParticlesPerNode(1, Node.Grid);
//        GenerateParticlesPerNode(N, Node.Grid);
//        return N;
//      }
//      i64 LeftN = GenerateParticles(
//        tree_node{
//          .Level = Node.Level,
//          .Height = u8(Node.Height + 1),
//          .NodeId = Node.NodeId * 2,
//          .Grid = SplitGrid(Node.Grid, Node.D, SpatialSplit, Left),
//          .D = i8((Node.D + 1) % Params.NDims)
//        }
//      );
//      i64 RightN = GenerateParticles(
//        tree_node{
//          .Level = Node.Level,
//          .Height = u8(Node.Height + 1),
//          .NodeId = Node.NodeId * 2 + 1,
//          .Grid = SplitGrid(Node.Grid, Node.D, SpatialSplit, Right),
//          .D = i8((Node.D + 1) % Params.NDims)
//        }
//      );
//      if (LeftN == 0 && RightN == 0) { // generate particles for the parent
//        GenerateParticlesPerNode(N, Node.Grid);
//        NCount += N;
//        REQUIRE(LeftN + RightN <= N);
//      } else if (LeftN > 0 && RightN == 0) { // generate particles for the right child
//        GenerateParticlesPerNode(N - LeftN, SplitGrid(Node.Grid, Node.D, SpatialSplit, Right));
//        REQUIRE(LeftN + RightN <= N);
//        NCount += N - LeftN;
//      } else if (LeftN == 0 && RightN > 0) { // generate particles for the left child
//        GenerateParticlesPerNode(N - RightN, SplitGrid(Node.Grid, Node.D, SpatialSplit, Left));
//        REQUIRE(LeftN + RightN <= N);
//        NCount += N - RightN;
//      } else {
//        if (Params.MaxParticleSubSampling <= 1)
//          REQUIRE(LeftN + RightN == N);
//      }
//    } 
//  } else if (Node.Height == Params.BaseHeight) { // refinement node, 1 children
////    if (GetNode(Node, &N) && N > 0) {
////      REQUIRE(N == 1);
////      i64 NNodesAtLeaf = NUM_NODES_AT_LEAF(Node.Level);
////      tree_node ChildNode = Node;
////      ChildNode.Height = Node.Height + 1;
////      ChildNode.NodeId = Node.NodeId + NNodesAtLeaf;
////      i8 D = Node.D; // NOTE: we won't be using ChildNode.D
////      vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
////      assert(Node.Grid.Dims3.x == 1 && Node.Grid.Dims3.y == 1 && Node.Grid.Dims3.z == 1);
////      bbox BBox{
////        .Min = Params.BBox.Min + Node.Grid.From3 * W3,
////        .Max = Params.BBox.Min + (Node.Grid.From3 + 1) * W3
////      };
////      u8 Left = 0;
////      while (ChildNode.Height <= Params.MaxHeight && GetRefNode(ChildNode, &Left)) {
////        float Half = (BBox.Max[D] + BBox.Min[D]) * 0.5f;
//////        printf("   level %d node %llu bit %d\n", Node.Level, Node.NodeId, Left);
////        if (Left) BBox.Max[D] = Half; else BBox.Min[D] = Half;
////        ChildNode.NodeId += NNodesAtLeaf;
////        ++ChildNode.Height;
////        D = i8((D + 1) % Params.NDims);
////      }
////      GenerateOneParticle(BBox);
////      ++NCount;
////    }
//  }
  return N;
}

// TODO: CONTINUE from here: fix the code so that we really refine by levels
// TODO: CONTINUE from here: re-implement the refine by error
// TODO: CONTINUE from here: output an .xyz
// TODO: CONTINUE from here: 
/* Write each level to a different file */
static void
FlushBlocksToFiles() {
  printf("--------- flushing blocks\n");
  /* write the resolution tree */
  FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, Params.NLevels), "wb");
  REQUIRE(Size(BlockStreams[Params.NLevels]) > 0);
  Flush(&BlockStreams[Params.NLevels]);
  fwrite(BlockStreams[Params.NLevels].Stream.Data, Size(BlockStreams[Params.NLevels]), 1, Fp);
  fclose(Fp);

  /* write the regular blocks */
  REQUIRE(BlockStreams.size() == Params.NLevels + 1);
  FOR(i8, L, 0, Params.NLevels) {
    WriteBlock(&BlockStreams[L], L, CurrBlocks[L]);
    if (Params.MaxHeight > Params.BaseHeight) { // flush refinement blocks
      u64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(L);
      FOR(u8, H, 0, Params.MaxHeight - Params.BaseHeight) {
        if (CurrRefBlocks[H].Level == L)
          WriteBlock(&RefBlockStreams[H], L, CurrRefBlocks[H].BlockId + (H + 1) * NBlocksAtLeaf);
      }
    }
    // write an index consisting of all blocks in the file
    // TODO: compress the index?
    // TODO: if too many blocks have 0 bytes, maybe we can write a sparse index
    FILE* Fp = fopen(PRINT("%s-%d.bin", Params.OutFile, L), "ab");
    Padding.resize(MaxBlockSize);
    fwrite(Padding.data(), Padding.size(), 1, Fp);
    u64 NBlocks = BlockBytes[L].size();
    fwrite(BlockBytes[L].data(), sizeof(block_meta) * NBlocks, 1, Fp);
    fwrite(&NBlocks, sizeof(NBlocks), 1, Fp);
    fwrite(&MaxBlockSize, sizeof(MaxBlockSize), 1, Fp);
    printf("max block size = %d\n", MaxBlockSize);
    fclose(Fp);
  }
  /* write the meta-data file */
  WriteMetaFile(Params, PRINT("%s.idx", Params.OutFile));
}

//bitstream* Bs = Height <= Params.BaseHeight ? &BlockStreams[Level] : &RefBlockStreams[Height - Params.BaseHeight - 1].Bs;
INLINE static void
EncodeNode(i8 Level, i64 NodeIdx, i64 M, i64 N) {
  // TODO: use binomial coding
  u64 BlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
//  printf("+++++++ encoding level = %d block = %llu\n", Level, BlockIdx);
  if (BlockIdx != CurrBlocks[Level]) { // we have moved to the next block, dump the current block to disk
    WriteBlock(&BlockStreams[Level], Level, CurrBlocks[Level]);
    CurrBlocks[Level] = BlockIdx;
  }
  bitstream* Bs = &BlockStreams[Level];
  GrowToAccomodate(Bs, 8);
//  WriteVarByte(Bs, N);
  EncodeCenteredMinimal((u32)N, (u32)M + 1, Bs);
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
//  WriteVarByte(&BlockStreams[Params.NLevels], N);
  EncodeCenteredMinimal((u32)N, (u32)M + 1, &BlockStreams[Params.NLevels]);
}

/* Encode particle refinement bits */
static void
EncodeParticle(i8 Level, u64 NodeIdx, const vec3f& Pos, bbox BBox) {
  //assert(BBox.Min.x <= Pos.x && BBox.Min.y <= Pos.y && BBox.Min.z <= Pos.z);
  //assert(BBox.Max.x >= Pos.x && BBox.Max.y >= Pos.y && BBox.Max.z >= Pos.z);
  vec3f P3 = Pos;
  u8 H = Params.BaseHeight + 1;
  i8 D = Params.BaseHeight % Params.NDims;
  u64 BaseBlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  while (H <= Params.MaxHeight) {
    i8 K = H - Params.BaseHeight - 1;
    if (CurrRefBlocks[K].BlockId == u64(-1))
      CurrRefBlocks[K].BlockId = BaseBlockIdx;
    if (CurrRefBlocks[K].Level == -1)
      CurrRefBlocks[K].Level = Level;
    //u64 BlockIdx = BaseBlockIdx + (K + 1) * NBlocksAtLeaf;
    bool NewBlock = BaseBlockIdx != CurrRefBlocks[K].BlockId;
    bool NewLevel = CurrRefBlocks[K].Level != Level;
    if (NewBlock || NewLevel) {
      i64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(CurrRefBlocks[K].Level);
      WriteBlock(&RefBlockStreams[K], CurrRefBlocks[K].Level, CurrRefBlocks[K].BlockId + (K + 1) * NBlocksAtLeaf);
      CurrRefBlocks[K].Level = Level;
      CurrRefBlocks[K].BlockId = BaseBlockIdx;
    }
    bitstream* Bs = &RefBlockStreams[K];
    GrowToAccomodate(Bs, 1);
    float Half = (BBox.Max[D] + BBox.Min[D]) * 0.5f;
    bool Left = P3[D] < Half;
    Write(Bs, Left);
//    printf("  level %d node %llu base block %llu ref block %llu bit %d\n", Level, NodeIdx, BaseBlockIdx, BaseBlockIdx + (K + 1) * NUM_BLOCKS_AT_LEAF(Level), Left);
    if (Left) BBox.Max[D] = Half;
    else BBox.Min[D] = Half;
    D = (D + 1) % Params.NDims;
    ++H;
  }
}

INLINE static void // N is the number of particles under a node
Print(i8 Level, u64 TreeIdx, i64 ResIdx, i64 LvlIdx, i64 ParIdx, i64 N) {
  printf("level = %d tree_idx = %llu res_idx = %lld lvl_idx = %lld par_idx = %lld N = %lld \n", Level, TreeIdx, ResIdx, LvlIdx, ParIdx, N);
  return;
}

struct Range {
  u64 From, To;
};

//std::vector<bbox> BBoxes; // one bounding box for each particle
//std::vector<Range> Ranges; // [level] -> from, to
//std::vector<u64> NodeIdxes; // one node index for each particle

// TODO: compute the total height using the accuracy and the global grid in the beginning
// TODO: write the refinement block and flush them to disk
//static void
//BuildTreeFineLevels() {
//  int ParticlesPerBlock = 2 * (1 << Params.BlockBits); // every refinement block is twice the size of the tree block
//  int BlockBytes = ParticlesPerBlock / 8; // one bit per particle
//  FOR (i8, L, 0, Params.NLevels) {
//  FOR (u64, I, Ranges[L].From, Ranges[L].To) {
//    i8 D = (Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z) % Params.NDims;
//    vec3f P3 = Particles[I].Pos;
//    bbox BBox = BBoxes[I];
//    u8 H = 0;
//    u64 BlockIdx = I / ParticlesPerBlock;
//    while (H < Params.Height) {
//      if (RBlockStreams.size() <= H) {
//        RBlockStreams.resize(H * 3 / 2 + 1);
//      }
//      bitstream* Bs = &RBlockStreams[H];
//      GrowToAccomodate(Bs, BlockBytes);
//      if (BlockIdx != CurrRBlocks[H]) {
//        // TODO: write the block to disk here
//        Rewind(Bs);
//        CurrRBlocks[H] = BlockIdx;
//      }
//      float Half = (BBox.Max[D] - BBox.Min[D]) * 0.5;
//      bool Left = (P3[D] - BBox.Min[D]) < Half;
//      Write(Bs, Left);
//      BBox.Max[D] = BBox.Max[D] - Half * Left;
//      BBox.Min[D] = BBox.Min[D] + Half * (1 - Left);
//      D = (D + 1) % 3;
//      ++H;
//    }
//  }}
//}

// TODO: what if we have 0 particles? should we stop the resolution divide?
static void
BuildTreeInner(q_item Q, float Accuracy) {
  std::queue<q_item> Queue;
  Queue.push(Q);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
  while (!Queue.empty()) {
    Q = Queue.front();
    Queue.pop();
    REQUIRE(Q.Height <= Params.MaxHeight);
    i64 N = Q.End - Q.Begin;
    assert((N == 1) || IS_EVEN(int(Q.Grid.Dims3[Q.D])));
    i64 Mid = Q.Begin;
    vec3f Error3 = (W3 * Q.Grid.Dims3) / f64(N);
    bool Stop = Error3.x <= Accuracy && Error3.y <= Accuracy;
    if (Params.NDims > 2) Stop = Stop && Error3.z <= Accuracy;
    if (Stop) continue;
    //if (N <= 1) continue; // enable this to stop the tree construction after the base height
    if (Q.SplitType == ResolutionSplit) { // resolution split
      auto Pred = [W3, &Q](const particle& P) {
        int Bin = MIN(Params.Dims3[Q.D] - 1, int((P.Pos[Q.D] - Params.BBox.Min[Q.D]) / W3[Q.D]));
        assert(IS_INT(Q.Grid.From3[Q.D]) && IS_INT(Q.Grid.Stride3[Q.D]) && IS_INT(Q.Grid.Dims3[Q.D]));
        REQUIRE((Bin - int(Q.Grid.From3[Q.D])) % int(Q.Grid.Stride3[Q.D]) == 0);
        Bin = (Bin - int(Q.Grid.From3[Q.D])) / int(Q.Grid.Stride3[Q.D]);
        return IS_EVEN(Bin);
//        float S = rand() * 1.0 / RAND_MAX;
//        return S < 0.5f;
      };
      Mid = partition(RANGE(Particles, Q.Begin, Q.End), Pred) - Particles.begin();
    } else { // spatial split
      float S = (Q.Grid.Dims3[Q.D] > 1.5f) * (Q.Grid.Stride3[Q.D] - 1) + 1;
      float M = Params.BBox.Min[Q.D] + W3[Q.D] * (Q.Grid.From3[Q.D] + Q.Grid.Dims3[Q.D] * 0.5f * S);
      auto Pred = [M, &Q](const particle& P) { return P.Pos[Q.D] < M; };
      Mid = partition(RANGE(Particles, Q.Begin, Q.End), Pred) - Particles.begin();
    }
    if (Q.Height < Params.BaseHeight) {
      /* encoding the children (left child in particular) */
      if (Q.SplitType == ResolutionSplit) {
        EncodeResNode(Q.End - Q.Begin, Mid - Q.Begin);
      } else {
        EncodeNode(Q.Level - (Q.SplitType == ResolutionSplit), Q.SplitType == ResolutionSplit ? Q.NodeIdx : Q.NodeIdx * 2, Q.End - Q.Begin, Mid - Q.Begin);
        //printf("%lld\n", Mid - Q.Begin);
        //printf("%lld\n", Q.End - Mid);
      }
      /* enqueue children */
      //Print(Q.Level - Q.RSplit, Q.TreeIdx * 2 + 1, Q.RSplit ? Q.ResIdx * 2 + 1 : Q.ResIdx, Q.RSplit ? Q.LvlIdx : Q.LvlIdx * 2 + 1, Q.ParIdx, Mid - Q.Begin); // encode only the left child
      if (Q.Begin < Mid) {
        Queue.push(q_item{
          .Begin = Q.Begin,
          .End = Mid,
          .TreeIdx = Q.TreeIdx * 2,
          .ResIdx = Q.SplitType == ResolutionSplit ? Q.ResIdx + 2 : Q.ResIdx,
          .NodeIdx = Q.SplitType == ResolutionSplit ? Q.NodeIdx : Q.NodeIdx * 2,
          .ParIdx = Q.ParIdx,
          .Grid = SplitGrid(Q.Grid, Q.D, Q.SplitType, side::Left),
          .D = i8((Q.D + 1) % Params.NDims),
          .Level = i8(Q.Level - (Q.SplitType == ResolutionSplit)),
          .Height = u8(Q.Height + 1),
          .SplitType = (N > 1 && (Q.SplitType == ResolutionSplit) && Q.Level > 1) ? ResolutionSplit : SpatialSplit
        });
      }
      if (Mid < Q.End) {
        Queue.push(q_item{
          .Begin = Mid,
          .End = Q.End,
          .TreeIdx = Q.TreeIdx * 2 + 1,
          .ResIdx = Q.SplitType == ResolutionSplit ? Q.ResIdx + 1 : Q.ResIdx,
          .NodeIdx = Q.SplitType == ResolutionSplit ? Q.NodeIdx : Q.NodeIdx * 2 + 1,
          .ParIdx = Q.ParIdx + Mid - Q.Begin,
          .Grid = SplitGrid(Q.Grid, Q.D, Q.SplitType, side::Right),
          .D = i8((Q.D + 1) % Params.NDims),
          .Level = Q.Level,
          .Height = u8(Q.Height + 1),
          .SplitType = SpatialSplit
        });
      }
    } else if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // Q.Height == Params.BaseHeight
      /* encoding the refinement bits */
      REQUIRE(N == 1);
      assert(Q.Grid.Dims3.x <= 1 && Q.Grid.Dims3.y <= 1 && Q.Grid.Dims3.z <= 1);
      // TODO: sometimes there are numerical issues where the particle is outside of BBox
      bbox BBox{
        .Min = Params.BBox.Min + Q.Grid.From3 * W3,
        .Max = Params.BBox.Min + (Q.Grid.From3 + Q.Grid.Dims3) * W3
      };
      EncodeParticle(Q.Level, Q.NodeIdx, Particles[Q.Begin].Pos, BBox);
    }
  }
}

static void
ComputeGrid(vec3i BBoxExt3, str DimsStr)
{
  i8 Depth = 0;
  i8 D = 0;
  while (BBoxExt3.x > 1 || BBoxExt3.y > 1 || BBoxExt3.z > 1) {
    if (BBoxExt3.x >= BBoxExt3.y && BBoxExt3.x >= BBoxExt3.z)
      D = 0;
    else if (BBoxExt3.y >= BBoxExt3.z && BBoxExt3.y >= BBoxExt3.x)
      D = 1;
    else if (BBoxExt3.z >= BBoxExt3.y && BBoxExt3.z >= BBoxExt3.x)
      D = 2;
    DimsStr[Depth++] = 'x' + D; 
    BBoxExt3[D] = (BBoxExt3[D] + 1) >> 1;
  }
}

static vec3i
ComputeGrid(
  std::vector<particle_int>* Particles, const bbox_int& BBox, 
  i64 Begin, i64 End, i8 Depth, str DimsStr)
{
  REQUIRE(Begin < End); // this cannot be a leaf node
  vec3i BBoxExt3 = BBox.Max - BBox.Min + 1;
  i8 D;
  if (BBoxExt3.x>=BBoxExt3.y && BBoxExt3.x>=BBoxExt3.z)
    D = 0;
  else if (BBoxExt3.y>=BBoxExt3.z && BBoxExt3.y>=BBoxExt3.x)
    D = 1;
  else if (BBoxExt3.z>=BBoxExt3.y && BBoxExt3.z>=BBoxExt3.x)
    D = 2;
  DimsStr[Depth] = 'x' + D; 
  assert((BBoxExt3[D]&1) == 0);
  i32 Middle = (BBox.Min[D]+BBox.Max[D]) >> 1;
  auto Pred = [D, Middle](const particle_int& P) { return P.Pos[D] <= Middle; };
  i64 Mid = std::partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left  = MCOPY(vec3i(0), [D]=1);
  vec3i LogDims3Right = MCOPY(vec3i(0), [D]=1);
  if (Begin+1 < Mid) {
    LogDims3Left = ComputeGrid(Particles, MCOPY(BBox, .Max[D]=Middle), Begin, Mid, Depth+1, DimsStr);
    ++LogDims3Left[D];
  }
  if (Mid+1 < End) {
    LogDims3Right = ComputeGrid(Particles, MCOPY(BBox, .Min[D]=Middle+1), Mid, End, Depth+1, DimsStr);
    ++LogDims3Right[D];
  }
  return max(LogDims3Left, LogDims3Right);
}

/* Return the dimensions of the underlying grid (in terms of power of two) */
static vec3i
ComputeGrid(
  std::vector<particle>* Particles, const bbox& BBox, 
  i64 Begin, i64 End, i8 Depth, str DimsStr)
{
  REQUIRE(Begin < End); // this cannot be a leaf node
  vec3f BBoxExt3 = Extent(BBox);
  i8 D;
  if (BBoxExt3.x >= BBoxExt3.y && BBoxExt3.x >= BBoxExt3.z)
    D = 0;
  else if (BBoxExt3.y >= BBoxExt3.z && BBoxExt3.y >= BBoxExt3.x)
    D = 1;
  else if (BBoxExt3.z >= BBoxExt3.y && BBoxExt3.z >= BBoxExt3.x)
    D = 2;
  DimsStr[Depth] = 'x' + D; 
  f32 Middle = (BBox.Min[D] + BBox.Max[D]) * 0.5f;
  auto Pred = [D, Middle](const particle& P) { return P.Pos[D] < Middle; };
  i64 Mid = std::partition(RANGE(*Particles, Begin, End), Pred) - Particles->begin();
  vec3i LogDims3Left  = MCOPY(vec3i(0), [D] = 1);
  vec3i LogDims3Right = MCOPY(vec3i(0), [D] = 1);
  if (Begin + 1 < Mid) {
    LogDims3Left = 
      ComputeGrid(Particles, MCOPY(BBox, .Max[D] = Middle), Begin, Mid, Depth + 1, DimsStr);
    ++LogDims3Left[D];
  }
  if (Mid + 1 < End) {
    LogDims3Right = 
      ComputeGrid(Particles, MCOPY(BBox, .Min[D] = Middle), Mid, End, Depth + 1, DimsStr);
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
  int NDims = Dims3.z == 1 ? 2 : 3;
  Err = std::sqrt(Err / (NDims * Particles2.size()));
  return Err;
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

//static f32
//Error2(
//  const bbox& BBox, 
//  const std::vector<particle>& Particles1, const std::vector<particle>& Particles2, const vec3i& Dims3)
//{
//  //bbox BBox = ComputeBoundingBox(Particles1);
//  vec3f W3 = (BBox.Max - BBox.Min) / vec3f(Dims3);
//  std::vector<std::vector<vec3f>> Grid(Dims3.x * Dims3.y * Dims3.z);
//  //std::vector<vec3f> Grid(Dims3.x * Dims3.y * Dims3.z, vec3f(NAN));
//  i32 ParticlesPerCell = MAX(1, Particles1.size() / (Dims3.x * Dims3.y * Dims3.z));
//  FOR_EACH(P, Particles1) {
//    vec3i Coord{
//      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
//      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
//      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
//    i32 Idx = Coord.z * (Dims3.x * Dims3.y) + Coord.y * (Dims3.x) + Coord.x;
//    //assert(Grid[Idx].x != Grid[Idx].x);
//    if (Grid[Idx].empty())
//      Grid[Idx].reserve(ParticlesPerCell);
//    Grid[Idx].push_back(P->Pos);
//  }
//  float Err = 0;
//  FOR_EACH(P, Particles2) {
//    vec3i Coord{
//      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
//      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
//      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
//    i32 MaxD = MAX(Dims3.z, MAX(Dims3.x, Dims3.y)) / 2;
//    for (i32 D = 0; D < MaxD; ++D) {
//      f32 MinDiff = INFINITY;
//      bool Found = false;
//      for (i32 Dz = -D; Dz <= D; ++Dz) {
//        i32 Z = Coord.z + Dz;
//        if (Z < 0 || Z >= Dims3.z) continue;
//        bool EdgeZ = (Dz == -D) || (Dz == D) || (Z == 0) || (Z == Dims3.z - 1);
//        for (i32 Dy = -D; Dy <= D; ++Dy) {
//          i32 Y = Dy + Coord.y;
//          if (Y < 0 || Y >= Dims3.y) continue;
//          bool EdgeY = (Dy == -D) || (Dy == D) || (Y == 0) || (Y == Dims3.y - 1);
//          bool Edge = EdgeZ || EdgeY;
//          for (i32 Dx = -D; Dx <= D; ++Dx) {
//            i32 X = Dx + Coord.x;
//            if (X < 0 || X >= Dims3.x) continue;
//            bool EdgeX = (Dx == -D) || (Dx == D) || (X == 0) || (X == Dims3.x - 1);
//            if (!(Edge || EdgeX)) continue;
//            i32 Idx = Z * (Dims3.x * Dims3.y) + Y * (Dims3.x) + X;
//            FOR_EACH() {
//              vec3f Diff = Grid[Idx] - P->Pos;
//              MinDiff = MIN(MinDiff, Diff.x * Diff.x + Diff.y * Diff.y + Diff.z * Diff.z);
//              Found = true;
//            }
//            if (Grid[Idx].x == Grid[Idx].x) { // not NAN
//            }
//          }
//        }
//      }
//      if (Found) {
//        Err += MinDiff;
//        break;
//      } else {
//        //assert(false);
//      }
//    }
//  }
//  int NDims = Dims3.z == 1 ? 2 : 3;
//  Err = std::sqrt(Err / (NDims * Particles2.size()));
//  return Err;
//}
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

static std::vector<particle_cell> ParticleCells;

struct bookkeeping_grid {
  std::vector<particle_cell> Cells;
  grid Grid;
};

static std::vector<bookkeeping_grid> BkGrids; // [level, depth] -> a grid of particle cells
//static std::vector<particle_cell> ParticleCells[128]; // [depth] -> vector of particle_cells

static void
TestCompressBlockZfp(f64 Accuracy, f64* BlockFloats, bitstream* Stream) {
  buffer_t BufFloats(BlockFloats, 64);
  buffer_t BufInts((i64*)BlockFloats, 64);
  u64 BlockUInts[4 * 4 * 4];
  buffer_t BufUInts (BlockUInts, 64);
  const i8 NBitPlanes = BIT_SIZE_OF(u64);
  const i8 Prec = NBitPlanes - 1 - 3; // 3 is the number of dimensions
  const i16 EMax = (i16)QuantizeF64(Prec, BufFloats, &BufInts);
  Write(Stream, EMax + traits<f64>::ExpBias, traits<f64>::ExpBits);
  ForwardZfp((i64*)BlockFloats, 3); // 3 = number of dimensions
  ForwardShuffle((i64*)BlockFloats, BlockUInts, 3); // 3 = number of dimensions
  GrowIfTooFull(Stream);
  i8 M = 0;
  for (int Bp = 64 - 1; Bp >= 0; --Bp) {
    i16 RealBp = Bp - EMax;
    bool TooHighPrecision = 5 - NBitPlanes + Bp < Exponent(Accuracy) - EMax;
    if (TooHighPrecision) break;
    //printf("encoding bit plane %d\n", Bp);
    Encode(BlockUInts, 64, Bp, M, Stream);
  }
}

static void
TestDecompressBlockZfp(f64 Accuracy, f64* BlockFloats, bitstream* Stream) {
  buffer_t BufFloats(BlockFloats, 64);
  buffer_t BufInts((i64*)BlockFloats, 64);
  u64 BlockUInts[64] = {};
  buffer_t BufUInts(BlockUInts, 64);
  i16 EMax = (i16)Read(Stream, traits<f64>::ExpBits) - traits<f64>::ExpBias;
  const i8 NBitPlanes = BIT_SIZE_OF(u64);
  i8 N = 0;
  for (int Bp = NBitPlanes - 1; Bp >= 0; --Bp) {
    bool TooHighPrecision = 5 - NBitPlanes + Bp < Exponent(Accuracy) - EMax;
    if (TooHighPrecision) break;
    //printf("decoding bit plane %d\n", Bp);
    Decode(BlockUInts, 64, Bp, N, Stream);
  }
  InverseShuffle(BlockUInts, (i64*)BlockFloats, 3); // 3 = number of dimensions
  InverseZfp((i64*)BlockFloats, 3); // 3 = number of dimensions
  const int Prec = NBitPlanes - 1 - 3; // 3 = number of dimensions
  Dequantize(EMax, Prec, BufInts, &BufFloats);
}

//static void
//TestCompressSeries(f64 Accuracy) {
//  /* generate a series of real values based on a grid */
//  i32 From = 0;
//  i32 Stride = 0.001;
//  i32 Dims = 64; // 100 blocks of 64-values
//  std::vector<f64> Vals(Dims);
//  for (int I = 0; I < Dims; ++I) {
//    f64 Low = From + I * Stride;
//    f64 High = From + (I + 1) * Stride;
//    f64 X = Low + (High - Low) * rand() / f64(RAND_MAX);
//    Vals[I] = X;
//  }
//  bitstream Stream;
//  InitWrite(&Stream, 100000);
//  f64 BlockFloats[64];
//  f64 BlockFloatsCopy[64];
//  FOR(i32, I, 0, 64) BlockFloats[I] = BlockFloatsCopy[I] = Vals[I];
//  TestCompressBlockZfp(Accuracy, BlockFloats, &Stream);
//  Flush(&Stream);
//  printf("stream size = %lld\n", BitSize(Stream));
//  InitRead(&Stream, Stream.Stream);
//  TestDecompressBlockZfp(Accuracy, BlockFloats, &Stream);
//  auto Err = RMSError(buffer_t<f64>(BlockFloatsCopy, 64), buffer_t<f64>(BlockFloats, 64));
//  //FOR(i32, I, 0, 64) {
//  //  printf("%f %f\n", BlockFloatsCopy[I], BlockFloats[I]);
//  //}
//  printf("rmse = %f\n", Err);
//}

f64 RMSE = 0;
i64 StreamSize = 0;
i64 NParticlesGenerated = 0;

static void
DecodeTreeDFS(i64 Begin, i64 End, u64 Code, const grid& Grid, i8 Level, split_type Split, i8 Depth) {
  i8 D = Params.DimsStr[Depth] - 'x';
  f32 W = (Params.BBox.Max[D] - Params.BBox.Min[D]) / Params.Dims3[D];
  vec3f GridDims3((f32)Params.Dims3.x, (f32)Params.Dims3.y, (f32)Params.Dims3.z);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / GridDims3;
  bbox ParentBBox{
    .Min = Params.BBox.Min + Grid.From3 * W3,
    .Max = Params.BBox.Min + (Grid.From3 + Grid.Dims3) * W3
  };
  bool Stop = Params.RefinementMode == refinement_mode::ERROR_BASED; // stop decoding
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > 2 * Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  i64 Mid = Begin;
  if (!Stop) {
    Mid = DecodeCenteredMinimal(u32(End - Begin + 1), &BlockStream) + Begin;
  } else { // stop going down in this branch
    GenerateParticlesPerNode(End - Begin, Grid, W3);
    NParticlesDecoded += End - Begin;
    NParticlesGenerated += End - Begin;
    return;
  }
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // left leaf
      ++NParticlesDecoded;
      auto G = SplitGrid(Grid, D, Split, side::Left);
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        vec3f Pos3;
        bbox BBox {
          .Min = Params.BBox.Min + G.From3 * W3,
          .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
        };
        /* decode the refinement bits (NOTE: either toggle this or the next block) */
        FOR(int, I, 0, 3) { // read refinement bits
          while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
            float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
            bool LeftSide = Read(&BlockStream);
            G.Dims3[I] *= 0.5f;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
          }
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
        }
        Particles.push_back(particle{ Pos3 });
      } else { // do not read the refinement bits
        GenerateParticlesPerNode(1, G, W3);
      }
    } else { // recurse on the left
      bool RSplit = Split == ResolutionSplit;
      split_type NextSplit = (RSplit && Level > 1) ? ResolutionSplit : SpatialSplit;
      DecodeTreeDFS(Begin, Mid, (Code * 2 + 1), SplitGrid(Grid, D, Split, side::Left), 
        Level - RSplit, NextSplit, Depth + 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // right leaf
      ++NParticlesDecoded;
      auto G = SplitGrid(Grid, D, Split, side::Right);
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        bbox BBox{
          .Min = Params.BBox.Min + G.From3 * W3,
          .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
        };
        /* decode the refinement bits (NOTE: either enable this or the next block) */
        vec3f Pos3;
        FOR(int, I, 0, 3) {
          while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
            float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
            bool LeftSide = Read(&BlockStream);
            G.Dims3[I] *= 0.5f;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
          }
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
        }
        Particles.push_back(particle{ Pos3 });
      } else { // do not read the refinement bits
        GenerateParticlesPerNode(1, G, W3);
      }
    } else { // recurse on the right
      DecodeTreeDFS(Mid, End, (Code * 2 + 2), SplitGrid(Grid, D, Split, side::Right), 
        Level, SpatialSplit, Depth + 1);
    }
  }
}

static cdf_table CdfTable;
static std::vector<cdf_table> BinomialTables;
static std::vector<std::vector<std::vector<f64>>> BinomialTablesF64;
static arithmetic_coder<> Coder;
static Rans64State Rans;

INLINE static void
EncodeNode(i64 NodeIdx, i64 M, i64 N) {
  bitstream* Bs = &BlockStream;
  GrowToAccomodate(Bs, 8);
  //EncodeCenteredMinimal((u32)N, (u32)M + 1, Bs);
  /* NOTE: comment out the following to disable the binomial coding */
  f64 Mean = f64(M) / 2; // mean
  f64 StdDev = sqrt(f64(M)) / 2; // standard deviation
  EncodeRange(Mean, StdDev, f64(0), f64(M), f64(N), CdfTable, &BlockStream, &Coder);
}

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

static void
BuildTreeInt(std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, split_type Split, i8 Depth) {
  i8 D = Params.DimsStr[Depth] - 'x';
  if (Grid.Dims3[D] == 1) {
    return BuildTreeInt(Particles, Begin, End, Grid, Split, Depth + 1);
  }
  i64 Mid = Begin;
  i32 MM = 0;
  if (Split == ResolutionSplit) { // resolution split
    auto RPred = [D, &Grid](const particle_int& P) {
      i32 Bin = P.Pos[D];
      REQUIRE((Bin - Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin - Grid.From3[D]) / Grid.Stride3[D];
      return IS_EVEN(Bin);
    };
    Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else { // spatial split
    MM = Grid.From3[D] + (((Grid.Dims3[D] + 1) >> 1) - 1) * Grid.Stride3[D];
    auto SPred = [MM, D, &Grid](const particle_int& P) {
      return P.Pos[D] <= MM;
    };
    Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }
  bbox_int ParentBBox {
    .Min = Grid.From3,
    .Max = Grid.From3 + (Grid.Dims3 - 1) * Grid.Stride3
  };
  bool Stop = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > 2 * Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  if (!Stop) {
    i64 M = End - Begin;
    i64 N = Mid - Begin;
    i64 P = M - N; // number of points on the right
    i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * Grid.Dims3.z;
    if (CellCount == M)
      return; // may speed up the encoding
    
    i64 CellCountRight = 1;
    FOR(int, I, 0, Params.NDims) {
      CellCountRight *= I == D ? (Grid.Dims3[I] >> 1) : Grid.Dims3[I];
    }
    REQUIRE(CellCount >= M);
    if (CellCount - M < M) { // more particles than empty cells, encode the empty cells
      ++NodesWithMoreParticlesCount;
      REQUIRE(CellCountRight >= P);
      //if (Split == SpatialSplit)
      //  EncodeCenteredMinimal(u32(CellCountRight - P), u32(CellCount - M + 1), &BlockStream);
      //else // resolution split, use binomial coding
      //  EncodeNode(0, CellCount - M, CellCountRight - P);
      if (CellCountRight - P <= 32) {
        ++Counts[CellCountRight - P];
      }
      if (CellCountRight - P == 0) {
        SeparationCodeLength += 1;
        //Write(&BlockStream, 0);
        ++ZeroCount;
        ++TotalCount;
        EncodeCenteredMinimal(u32(CellCountRight - P), u32(CellCount - M + 1), &BlockStream);
      } else {
        EncodeCenteredMinimal(u32(CellCountRight - P), u32(CellCount - M + 1), &BlockStream);
        SeparationCodeLength += log2(CellCount - M +1);
        ++TotalCount;
      }
      //SeparationCodeLength += log2(f64(CellCount - M + 1));
      if (M < CellCount) {
        Ratio += (f64(CellCount - P) / (CellCount - M));
        RatioCount++;
      }
    } else { // more empty cells than particles, encode the particles
      ++NodesWithMoreEmptyCellsCount;
      //if (Split == SpatialSplit)
      //  EncodeCenteredMinimal(u32(P), u32(M + 1), &BlockStream);
      //else // resolution split, use binomial coding
      //  EncodeNode(0, M, P);
      if (P <= 32) {
        ++Counts[P];
      }
      if (P == 0) {
        SeparationCodeLength += 1;
        ++ZeroCount;
        ++TotalCount;
        //Write(&BlockStream, 0);
        EncodeCenteredMinimal(u32(P), u32(M + 1), &BlockStream);
      } else {
        EncodeCenteredMinimal(u32(P), u32(M + 1), &BlockStream);
        SeparationCodeLength += log2(M+1);
        ++TotalCount;
      }
      //SeparationCodeLength += log2(M + 1);
      Ratio += (f64(N) / M);
      RatioCount++;
    }
  } else {
    return;
  }
  //if (Begin < Mid) {
  //  Write(&BlockStream, 1);
  //} else {
  //  Write(&BlockStream, 0);
  //}
  //if (Mid < End) {
  //  Write(&BlockStream, 1);
  //} else {
  //  Write(&BlockStream, 0);
  //}
  /* recurse on the left or right */
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // left leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // write refinement bits
        vec3i P3 = Particles[Begin].Pos;
        vec3i Pos3;
        auto G = SplitGrid(Grid, D, Split, side::Left);
        bbox_int BBox {
          .Min = G.From3,
          .Max = G.From3 + (G.Dims3 - 1) * G.Stride3
        };
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
            REQUIRE(BBox.Min[I] <= P3[I]);
            REQUIRE(BBox.Max[I] >= P3[I]);
            G.Dims3[I] = (G.Dims3[I] + 1) >> 1;
            i32 Half = G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I];
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] = BBox.Min[I] = Half + G.Stride3[I];
            }
          }
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
          ++NParticlesDecoded;
        }
      }
    } else { // recurse on the left
      split_type NextSplit = Grid.Dims3.x * Grid.Dims3.y * Grid.Dims3.z > THRESHOLD ? SpatialSplit : ResolutionSplit;
      BuildTreeInt(Particles, Begin, Mid, SplitGrid(Grid, D, Split, side::Left), NextSplit, Depth + 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // right leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        vec3i Pos3;
        vec3i P3 = Particles[Mid].Pos;
        auto G = SplitGrid(Grid, D, Split, side::Right);
        bbox_int BBox {
          .Min = G.From3,
          .Max = G.From3 + (G.Dims3 - 1) * G.Stride3
        };
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          while (G.Dims3[I] > 1/* && BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy*/) { // NOTE: enable this to refine the tree uniformly until the leaf level
            REQUIRE(BBox.Min[I] <= P3[I]);
            REQUIRE(BBox.Max[I] >= P3[I]);
            G.Dims3[I] = (G.Dims3[I] + 1) >> 1;
            i32 Half = G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I];
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] = BBox.Min[I] = Half + G.Stride3[I];
            }
          }
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
          ++NParticlesDecoded;
        }
      }
    } else { // recurse on the right
      split_type NextSplit = Grid.Dims3.x * Grid.Dims3.y * Grid.Dims3.z > THRESHOLD ? SpatialSplit : ResolutionSplit;
      BuildTreeInt(Particles, Mid, End, SplitGrid(Grid, D, Split, side::Right), NextSplit, Depth + 1);
    }
  }
}

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

/* In the first pass, we collect statistics about node value distribution */
static void
BuildTreeIntPass1(std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, split_type Split, i8 Depth) {
  i64 N = End - Begin; // total number of particles
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  if (CellCount == N) return;

  bbox_int ParentBBox {
    .Min = Grid.From3,
    .Max = Grid.From3 + (Grid.Dims3 - 1) * Grid.Stride3
  };
  i8 D = 0;
  if (Grid.Dims3[1] > Grid.Dims3[D]) D = 1;
  if (Grid.Dims3[2] > Grid.Dims3[D]) D = 2;

  i32 MM = Grid.From3[D] + (((Grid.Dims3[D] + 1) >> 1) - 1) * Grid.Stride3[D];
  auto SPred = [MM, D, &Grid](const particle_int& P) {
    return P.Pos[D] <= MM;
  };
  i64 Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();

  bool EnoughAccuracy = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > 2 * Params.Accuracy) {
      EnoughAccuracy = false;
      break;
    }
  }
  if (EnoughAccuracy) return;

  i64 P = End - Mid; // number of particles on the right
  auto BBoxRight = MCOPY(ParentBBox, .Min[D] = MM + Grid.Stride3[D]); // bbox of the right child
  auto GR = (BBoxRight.Max - BBoxRight.Min) / Grid.Stride3 + 1; // grid of the right child
  i64 CellCountRight = i64(GR.x) * i64(GR.y) * i64(GR.z);
  if (CellCount - N < N) {
    N = CellCount - N;
    P = CellCountRight - P;
  }
  N = MIN(N, CellCountRight);
  if (N <= Threshold)
    ++ProbCount[Depth][N][P];

  if (Begin + 1 < Mid) 
    BuildTreeIntPass1(Particles, Begin, Mid, SplitGrid(Grid, D, Split, side::Left), SpatialSplit, Depth + 1);
  if (Mid + 1 < End)
    BuildTreeIntPass1(Particles, Mid, End, SplitGrid(Grid, D, Split, side::Right), SpatialSplit, Depth + 1);
}

static void
BuildTreeIntPass2(std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, split_type Split, i8 Depth) {
  i64 N = End - Begin; // total number of particles
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * Grid.Dims3.z;
  if (CellCount == N) { NParticlesDecoded += N; return; };

  bbox_int ParentBBox {
    .Min = Grid.From3,
    .Max = Grid.From3 + (Grid.Dims3 - 1) * Grid.Stride3
  };
  i8 D = 0;
  if (Grid.Dims3[1] > Grid.Dims3[D]) D = 1;
  if (Grid.Dims3[2] > Grid.Dims3[D]) D = 2;

  i32 MM = Grid.From3[D] + (((Grid.Dims3[D] + 1) >> 1) - 1) * Grid.Stride3[D];
  auto SPred = [MM, D, &Grid](const particle_int& P) {
    return P.Pos[D] <= MM;
  };
  i64 Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();

  bool EnoughAccuracy = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > 2 * Params.Accuracy) {
      EnoughAccuracy = false;
      break;
    }
  }
  if (EnoughAccuracy) return;

  i64 P = End - Mid; // number of particles on the right
  auto BBoxRight = MCOPY(ParentBBox, .Min[D] = MM + Grid.Stride3[D]); // bbox of the right child
  auto GR = (BBoxRight.Max - BBoxRight.Min) / Grid.Stride3 + 1; // grid of the right child
  i64 CellCountRight = i64(GR.x) * i64(GR.y) * i64(GR.z);
  REQUIRE(CellCount >= N);
  if (CellCount - N < N) {
    N = CellCount - N;
    P = CellCountRight - P;
  }
  N = MIN(N, CellCountRight);
  if (N <= Threshold) {
    ArithmeticEncode(N, P, ProbCount[Depth][N][N], ProbCount[Depth][N], &Coder);
    //ArithmeticEncode(M, P, ProbTotal, SingleProbCount, &Coder);
    f64 R = P == 0 ? ProbCount[Depth][N][0] : ProbCount[Depth][N][P] - ProbCount[Depth][N][P-1];
    REQUIRE(R > 0);
    SeparationCodeLength += log2(f64(ProbCount[Depth][N][N]) / R);
  } else {
    EncodeCenteredMinimal(u32(P), u32(N + 1), &BlockStream);
    SeparationCodeLength += log2(N + 1);
  }

  /* recurse on the left or right */
  if (Begin + 1 == Mid && Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
    vec3i P3 = Particles[Begin].Pos;
    vec3i Pos3;
    auto G = SplitGrid(Grid, D, Split, side::Left);
    bbox_int BBox {
      .Min = G.From3,
      .Max = G.From3 + (G.Dims3 - 1) * G.Stride3
    };
    FOR(int, I, 0, Params.NDims) { // go until one particle per cell
      while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
        REQUIRE(BBox.Min[I] <= P3[I]);
        REQUIRE(BBox.Max[I] >= P3[I]);
        G.Dims3[I] = (G.Dims3[I] + 1) >> 1;
        i32 Half = G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I];
        bool LeftSide = P3[I] <= Half;
        Write(&BlockStream, LeftSide);
        ++RefinementCodeLength;
        if (LeftSide) BBox.Max[I] = Half;
        else G.From3[I] = BBox.Min[I] = Half + G.Stride3[I];
      }
      REQUIRE(BBox.Min[I] <= P3[I]);
      REQUIRE(BBox.Max[I] >= P3[I]);
      Pos3[I] = (BBox.Max[I] + BBox.Min[I]) >> 1;
      auto Diff = Pos3[I] - P3[I];
      RMSE += Diff * Diff;
    }
    ++NParticlesDecoded;
  } else if (Begin + 1 < Mid) {
    BuildTreeIntPass2(Particles, Begin, Mid, SplitGrid(Grid, D, Split, side::Left), SpatialSplit, Depth + 1);
  }

  if (Mid + 1 == End && Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // right leaf
    vec3i Pos3;
    vec3i P3 = Particles[Mid].Pos;
    auto G = SplitGrid(Grid, D, Split, side::Right);
    bbox_int BBox {
      .Min = G.From3,
      .Max = G.From3 + (G.Dims3 - 1) * G.Stride3
    };
    FOR(int, I, 0, Params.NDims) { // go until one particle per cell
      while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
        REQUIRE(BBox.Min[I] <= P3[I]);
        REQUIRE(BBox.Max[I] >= P3[I]);
        G.Dims3[I] = (G.Dims3[I] + 1) >> 1;
        i32 Half = G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I];
        bool LeftSide = P3[I] <= Half;
        Write(&BlockStream, LeftSide);
        ++RefinementCodeLength;
        if (LeftSide) BBox.Max[I] = Half;
        else G.From3[I] = BBox.Min[I] = Half + G.Stride3[I];
      }
      REQUIRE(BBox.Min[I] <= P3[I]);
      REQUIRE(BBox.Max[I] >= P3[I]);
      Pos3[I] = (BBox.Max[I] + BBox.Min[I]) >> 1;
      auto Diff = Pos3[I] - P3[I];
      RMSE += Diff * Diff;
    }
    ++NParticlesDecoded;
  } else if (Mid + 1 < End) { // recurse on the right
    BuildTreeIntPass2(Particles, Mid, End, SplitGrid(Grid, D, Split, side::Right), SpatialSplit, Depth + 1);
  }
}

static i64 CodeLengthPerLevel[32] = {};
/* Try both the balance and the spatial split and choose the best one */
static void
BuildTreeIntTryBoth(std::vector<particle_int>& ParticlesInt, i64 Begin, i64 End, bbox_int BBox, i8 Depth) {
  auto G = BBox.Max - BBox.Min + 1; // grid formed by BBox
  i8 D = 0;
  if (G[1] > G[D])
    D = 1;
  if (G[2] > G[D])
    D = 2;

  i64 Mid1, Mid2 = Begin;
  i32 MM1, MM2 = BBox.Min[D];
  auto Pred = [D](const particle_int& P1, const particle_int& P2) {
    return P1.Pos[D] < P2.Pos[D];
  };
  std::sort(ParticlesInt.begin() + Begin, ParticlesInt.begin() + End, Pred);
  Mid1 = (Begin + End + 1) >> 1; // balance
  MM1 = ParticlesInt[Mid1-1].Pos[D]; // split into [B - M], [M - E)

  MM2 = (BBox.Min[D] + BBox.Max[D]) >> 1; // spatial
  auto SPred = [MM2, D](const particle_int& P) {
    return P.Pos[D] <= MM2;
  };
  Mid2 = std::stable_partition(RANGE(ParticlesInt, Begin, End), SPred) - ParticlesInt.begin();
  bool Stop = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  if (Stop)
    return;
    // encode the bin where Particle[Mid].Pos[D] is
  i64 N  = End - Begin; // total number of particles
  i64 M1 = Mid1 - Begin, M2 = Mid2 - Begin; // number of particles on the left
  i64 P1 = End - Mid1, P2 = End - Mid2; // number of particles on the right
  i64 CellCount = i64(G.x) * i64(G.y) * i64(G.z);
  i64 CellCountLeft1 = 1, CellCountLeft2 = 1;
  i64 CellCountRight1 = 1, CellCountRight2 = 1;
  { // balance, left
    auto BBoxCopy = BBox;
    BBoxCopy.Max[D] = ParticlesInt[Mid1-1].Pos[D];
    auto GR = BBoxCopy.Max - BBoxCopy.Min + 1;
    CellCountLeft1 = i64(GR.x) * i64(GR.y) * i64(GR.z);
  }
  { // balance, right
    auto BBoxCopy = BBox;
    BBoxCopy.Min[D] = ParticlesInt[Mid1-1].Pos[D];
    auto GR = BBoxCopy.Max - BBoxCopy.Min + 1;
    CellCountRight1 = i64(GR.x) * i64(GR.y) * i64(GR.z);
  }
  { // spatial, left
    auto BBoxCopy = BBox;
    BBoxCopy.Max[D] = MM2;
    auto GR = BBoxCopy.Max - BBoxCopy.Min + 1;
    CellCountLeft2 = i64(GR.x) * i64(GR.y) * i64(GR.z);
  }
  { // spatial, right
    auto BBoxCopy = BBox;
    BBoxCopy.Min[D] = MM2 + 1;
    auto GR = BBoxCopy.Max - BBoxCopy.Min + 1;
    CellCountRight2 = i64(GR.x) * i64(GR.y) * i64(GR.z);
  }
  if (CellCount - N < N) { // more particles than empty cells, encode the empty cells
    N = CellCount - N;
    assert(CellCountLeft1 >= M1);
    assert(CellCountLeft2 >= M2);
    assert(CellCountRight1 >= P1);
    assert(CellCountRight2 >= P2);
    P1 = CellCountRight1 - P1;
    P2 = CellCountRight2 - P2;
    M1 = CellCountLeft1 - M1;
    M2 = CellCountLeft2 - M2;
  }
  if (N == 0) {
    NParticlesDecoded += End - Begin;
    return;
  }
 
  //f64 Score1 = fabs(f64(M1) / CellCountLeft1 - f64(P1) / CellCountRight1); // balance
  //f64 Score2 = fabs(f64(M2) / CellCountLeft2 - f64(P2) / CellCountRight2); // spatial
  //f64 Score1 = (f64(M1) / CellCountLeft1 + f64(P1) / CellCountRight1); // balance
  //f64 Score2 = (f64(M2) / CellCountLeft2 + f64(P2) / CellCountRight2); // spatial
  f64 Score1 = MIN(f64(M1) / CellCountLeft1, f64(P1) / CellCountRight1); // balance
  f64 Score2 = MIN(f64(M2) / CellCountLeft2, f64(P2) / CellCountRight2); // spatial
  //f64 Score1 = fabs(1.0 - f64(CellCountLeft1) / CellCountRight1); // balance
  //f64 Score2 = fabs(1.0 - f64(M2) / (P2)); // spatial
  split_type Split;
  i32 M;
  i32 MM;
  i64 Mid;
  i64 P;
  if (false) {
  //if (Score1 < Score2) {
    Split = BalanceSplit;
    Mid = Mid1;
    P = P1;
    MM = MM1;
  } else {
    Split = SpatialSplit;
    Mid = Mid2;
    P = P2;
    MM = MM2;
  }

  if (Split == BalanceSplit) {
    M = ParticlesInt[Mid-1].Pos[D]; // split into [B - M], [M - E)
    assert(BBox.Min[D] <= M);
    assert(BBox.Max[D] >= M);
    EncodeCenteredMinimal(M - BBox.Min[D], G[D], &BlockStream);
    //SeparationCodeLength += log2(G[D]);
    CodeLengthPerLevel[Depth] += log2(G[D]);
  } else if (Split == SpatialSplit) {
    //assert(N <= CellCountRight2);
    if (G.x * G.y * G.z > 16) {
      //SeparationCodeLength += log2(N + 1);
      EncodeCenteredMinimal(u32(P), (MIN(u32(N), u32(CellCountRight2))) + 1, &BlockStream);
    } else {
      REQUIRE(G.x * G.y * G.z == 16);
      if (M2 > 0)
        SeparationCodeLength += 8;
      if (P2 > 0)
        SeparationCodeLength += 8;
      Write(&BlockStream, 0, 16);
      return;
    }
    CodeLengthPerLevel[Depth] += log2(N + 1);
  }
  /* recurse on the left or right */
  if (Begin < Mid) {
    auto BBoxCopy = BBox;
    if (Split == BalanceSplit) {
      BBoxCopy.Max[D] = M;
    } else if (Split == SpatialSplit) {
      BBoxCopy.Max[D] = MM;
    }
    if (Begin + 1 == Mid) { // left leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // write refinement bits
        vec3i P3 = ParticlesInt[Begin].Pos;
        vec3i Pos3;
        FOR(int, I, 0, Params.NDims) {
          while (BBoxCopy.Max[I] - BBoxCopy.Min[I] > 2 * Params.Accuracy) {
            REQUIRE(BBoxCopy.Min[I] <= P3[I]);
            REQUIRE(BBoxCopy.Max[I] >= P3[I]);
            i32 Half = (BBoxCopy.Min[I] + BBoxCopy.Max[I]) >> 1;
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBoxCopy.Max[I] = Half;
            } else {
              BBoxCopy.Min[I] = Half + 1;
            }
          }
          Pos3[I] = (BBoxCopy.Max[I] + BBoxCopy.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];  
          RMSE += Diff * Diff;
        }
        ++NParticlesDecoded;
      }
    } else { // recurse on the left
      BuildTreeIntTryBoth(ParticlesInt, Begin, Mid, BBoxCopy, Depth + 1);
    }
  }

  if (Mid < End) {
    auto BBoxCopy = BBox;
    if (Split == BalanceSplit) {
      BBoxCopy.Min[D] = M;
    } else if (Split == SpatialSplit) {
      BBoxCopy.Min[D] = MM + 1;
    }
    if (Mid + 1 == End) { // right leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        vec3i Pos3;
        vec3i P3 = ParticlesInt[Mid].Pos;
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          while (BBoxCopy.Max[I] - BBoxCopy.Min[I] > 2 * Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
            REQUIRE(BBoxCopy.Min[I] <= P3[I]);
            REQUIRE(BBoxCopy.Max[I] >= P3[I]);
            i32 Half = (BBoxCopy.Min[I] + BBoxCopy.Max[I]) >> 1;
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBoxCopy.Max[I] = Half;
            } else {
              BBoxCopy.Min[I] = Half + 1;
            }
          }
          Pos3[I] = (BBoxCopy.Max[I] + BBoxCopy.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
        }
        ++NParticlesDecoded;
      }
    } else { // recurse on the right
      BuildTreeIntTryBoth(ParticlesInt, Mid, End, BBoxCopy, Depth + 1);
    }
  }
}

static tree* TreePtr = nullptr;

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
  int begin = pow(2, level) - 1;
  int end = level+1<nlevels ? begin*2+1 : begin+nleaves;
  return vec2i{begin, end};
}

static std::vector<i32>
BuildBinaryTreeForResiduals(const std::vector<i32>& Residuals) {
  if (Residuals.size() == 0)
    return std::vector<i32>();
  int n = (int)Residuals.size();
  int nodes = 0;
  int k = 1;
  int nlevels = 1;
  while (k < n) {
    nodes += k;
    k *= 2;
    ++nlevels;
  }
  int nleaves = k>n ? (2*n-k) : k;
  nodes += nleaves;

  /* allocate the buffer and copy the contents */
  std::vector<i32> ResidualTree(nodes);
  for (int I = nodes-nleaves, J=n-nleaves; I < nodes; ++I, ++J) {
    ResidualTree[I] = Residuals[J]; // copy the nleaves
  }
  for (int I = nodes-n, J=0; I < nodes-nleaves; ++I, ++J) {
    ResidualTree[I] = Residuals[J];
  }
  auto be = IndexRange(nlevels-1, nlevels, nleaves);
  int begin = be[0], end = be[1];
  while (begin > 0) {
    assert((end-begin)%2 == 0);
    for (int i = begin; i < end; i += 2) {
      int p = (i-1) >> 1;
      ResidualTree[p] = ResidualTree[i] + ResidualTree[i+1];
    }
    end = begin;
    begin = (begin-1) >> 1;
  }
  int Sum = 0;
  for (int I = 0; I < Residuals.size(); ++I) {
    Sum += Residuals[I];
  }
  assert(Sum == ResidualTree[0]);
  return ResidualTree;
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
#define BINOMIAL 1
/* At certain depth, we split the node using the Resolution split into a number of levels, then use the
low-resolution nodes to predict the values for finer-resolution nodes */
static tree*
DecodeTreeIntPredict(const tree* PredNode,
  std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, 
  split_type Split, i8 ResLvl, i8 Depth) 
{
  /* early return if the number of particles is the same as the number of cells */
  i64 N = End - Begin; // total number of particles
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);

  /* determine the splitting axis */
  bbox_int ParentBBox {
    .Min = Params.BBoxInt.Min + Grid.From3,
    .Max = Params.BBoxInt.Min + Grid.From3 + (Grid.Dims3 - 1) * Grid.Stride3
  };
  i8 D = Params.DimsStr[Depth] - 'x';
  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  assert(CellCountLeft+CellCountRight == CellCount);

  /* decode to find Mid */
#if !defined(BINOMIAL)
  i64 Mid = Begin;
  i64 P = End - Mid;
  bool Flip = false;
  if (CellCount-N < N) {
    Flip = true;
    N = CellCount - N;
  }
  //N = MIN(N, CellCountRight);
  P = DecodeCenteredMinimal(u32(N+1), &BlockStream);
  if (Flip) {
    P = CellCountRight - P;
  }
  Mid = End - P;
#else
  i64 Mid = Begin;
  i64 P = Mid - Begin;
  bool DecodeEmptyCells = CellCount-N < N;
  if (DecodeEmptyCells)
    N = CellCount - N;
  if (PredNode) { // predict P
    i64 M = PredNode->Count;
    i64 K = PredNode->Left?PredNode->Left->Count : M - PredNode->Right->Count;
    f64 Prob = DecodeEmptyCells ? 1-ProbBin(M, K) : ProbBin(M, K);
    u32 L = u32(Prob * (N+1));
    assert(L <= N);
    if (N <= BinomialCutoff && N>0) {
      const cdf& Cdf = BinomialTables[N][L];
      P = DecodeBinomialSmallRange(N, Cdf, &Coder);
    } else if (N > 0) { // 
      f64 Mean = N * Prob;
      f64 StdDev = sqrt(N*Prob*(1-Prob));
      P = DecodeRange(Mean, StdDev, f64(0), f64(N), BinomialTables[0], &BlockStream, &Coder);
    }
  } else if (N > 0) { // encode as normal
    P = DecodeCenteredMinimal(u32(N+1), &BlockStream);
  }
  P = DecodeEmptyCells ? CellCountLeft-P : P;
  Mid = P + Begin;
#endif

  /* recurse */
  tree* Left = nullptr; 
  if (Begin+1==Mid && CellCountLeft==1) {
    // TODO
    assert(Depth+1 == Params.MaxDepth);
    Left = new (TreePtr++) tree;
    Left->Count = 1;
    ++NParticlesDecoded;
    particle_int Particle{.Pos = Params.BBoxInt.Min + GridLeft.From3};
    Particles.push_back(Particle);
  } else if (Begin < Mid) { // recurse
    assert(Depth+1 < Params.MaxDepth);
    //split_type NextSplit = 
    //  ((Depth+1==Params.StartResolutionSplit) ||
    //   (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
    //if (Split == SpatialSplit)
    //  Left = DecodeTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
    //else if (Split == ResolutionSplit)
    //  Left = DecodeTreeIntPredict(nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
    Left = DecodeTreeIntPredict(nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);      
  }

  /* recurse on the right */
  tree* Right = nullptr;
  if (Mid+1==End && CellCountRight==1) {
    assert(Depth+1 == Params.MaxDepth);
    Right = new (TreePtr++) tree;
    Right->Count = 1;
    particle_int Particle{.Pos = Params.BBoxInt.Min + GridRight.From3};
    Particles.push_back(Particle);
    ++NParticlesDecoded;
  } else if (Mid < End) { //recurse
    assert(Depth+1 < Params.MaxDepth);
    //split_type NextSplit = SpatialSplit;
    //if (Split == SpatialSplit)
    //  Right = DecodeTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    //else if (Split == ResolutionSplit)
    //  Right = DecodeTreeIntPredict(Left, Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit) ? ResolutionSplit : SpatialSplit;
    Right = DecodeTreeIntPredict(nullptr, Particles, Mid, End, GridRight, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);
  }

  if (Depth < Params.MaxDepth) {
    assert(Left || Right);
  }

  /* construct the prediction tree */
  tree* Node = nullptr;
  //if (Split == ResolutionSplit) {
  //  Node = BuildPredTree(Left, Right, Depth, D);
  //} else if (Split == SpatialSplit) {
  //  Node = new (TreePtr++) tree;
  //  Node->Left  = Left;
  //  Node->Right = Right;
  //  if (Left ) Node->Count = Left->Count; else Node->Count = 0;
  //  if (Right) Node->Count += Right->Count;
  //}
  // TODO: deallocate the two merged trees

  return Node;
}

static std::vector<i32> Residuals;
static std::vector<std::vector<particle_int>> ParticleLevels;

/* At certain depth, we split the node using the Resolution split into a number of levels, then use the
low-resolution nodes to predict the values for finer-resolution nodes */
static tree*
BuildTreeIntPredict(const tree* PredNode,
  std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, 
  split_type Split, i8 ResLvl, i8 Depth) 
{
  /* early return if the number of particles is the same as the number of cells */
  i64 N = End - Begin; // total number of particles
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  //if (CellCount == N) return nullptr;

  /* determine the splitting axis */
  bbox_int ParentBBox {
    .Min = Params.BBoxInt.Min + Grid.From3,
    .Max = Params.BBoxInt.Min + Grid.From3 + (Grid.Dims3-1) * Grid.Stride3
  };
  i8 D = Params.DimsStr[Depth] - 'x';

  /* split in either resolution or precision */
  i64 Mid = Begin;
  i32 MM = Grid.From3[D]; // the beginning of the right child
  if (Split == ResolutionSplit) {
    auto RPred = [D, &Grid](const particle_int& P) {
      i32 Bin = P.Pos[D] - Params.BBoxInt.Min[D];
      //REQUIRE((Bin-Grid.From3[D]) % Grid.Stride3[D] == 0);
      Bin = (Bin-Grid.From3[D]) / Grid.Stride3[D];
      return IS_EVEN(Bin);
    };
    Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else if (Split == SpatialSplit) {
    MM = Params.BBoxInt.Min[D] + Grid.From3[D] + (((Grid.Dims3[D]+1)>>1)-1) * Grid.Stride3[D];
    auto SPred = [MM, D, &Grid](const particle_int& P) {
      return P.Pos[D] <= MM;
    };
    Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }

  /* encode */
  auto GridLeft  = SplitGrid(Grid, D, Split, side::Left );
  auto GridRight = SplitGrid(Grid, D, Split, side::Right);
  i64 CellCountRight = i64(GridRight.Dims3.x) * i64(GridRight.Dims3.y) * i64(GridRight.Dims3.z);
  i64 CellCountLeft  = i64(GridLeft .Dims3.x) * i64(GridLeft .Dims3.y) * i64(GridLeft .Dims3.z);
  REQUIRE(CellCountLeft+CellCountRight == CellCount);
#if !defined(BINOMIAL)
  i64 P = End - Mid;
  if (CellCount-N < N) {
    N = CellCount - N;
    P = CellCountRight - P;
  }
  //N = MIN(N, CellCountRight); // this only makes sense if the grid dimension is non power of two (so that the right can have fewer cells than the left)
  EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
  BinomialCodeSize += log2(N+1);
#else
  i64 P = Mid - Begin; // number of particles on the left
  bool EncodeEmptyCells = CellCount-N < N;
  if (EncodeEmptyCells) {
    N = CellCount - N;
    P = CellCountLeft - P;
  }
  if (PredNode) { // predict P
    //i64 M = PredNode->Count;
    //i64 K = PredNode->Left?PredNode->Left->Count : M - PredNode->Right->Count;
    //f64 Prob = EncodeEmptyCells ? 1-ProbBin(M, K) : ProbBin(M, K); // probability of throwing particle onto the left
    //u32 L = u32(Prob * (N+1));
    //assert(L <= N);
    //PredictedNodeCount++;
    //i32 R = i32(L) - i32(P); // residual
    //R = R<0 ? -(2*R+1) : 2*R;
    //ResidualCodeLengthNormal += log2(N+1);
    //ResidualCodeLengthGamma += 2*floor(log2(R+1))+1;
    //if (N<=BinomialCutoff && N>0) {
    //  const cdf& Cdf = BinomialTables[N][L];
    //  //DebugProbs.push_back(debug_prob{u32(P), u32(N), u32(L)});
    //  EncodeBinomialSmallRange(N, P, Cdf, &Coder);
    //  //EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
    //} else if (N > 0) { // N > BinomialCutoff
    //  f64 Mean = N * Prob;
    //  f64 StdDev = sqrt(N*Prob*(1-Prob));
    //  //f64 BitCount = EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), BinomialTables[0], &BlockStream, &Coder);
    //  int Stop = 0;
    //  //EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
    //  Residuals.push_back(R);
    //}
  } else if (Split==ResolutionSplit && N>1) {
    //const cdf& Cdf = CdfTable[N];
    //EncodeBinomialSmallRange(N, P, Cdf, &Coder);
    EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
    //f64 Mean = f64(N) / 2; // mean
    //f64 StdDev = sqrt(f64(N)) / 2; // standard deviation
    //EncodeRange(Mean, StdDev, f64(0), f64(N), f64(P), CdfTable, &BlockStream, &Coder);
    //EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
    //i32 R = N/2 - P;
    //R = R<0 ? -(2*R+1) : 2*R;
    //Residuals.push_back(R);
  } else if (N > 0) { // encode as normal
    EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);
    //NonPredictedNodeCount++;
    //NonPredictedCodeSize += log2(N+1);
  }
#endif

  if (Depth+1 == Params.StartResolutionSplit) {
    static bool Done = false;
    if (!Done) {
      printf("grid dims is %d %d %d\n", Grid.Dims3.x, Grid.Dims3.y, Grid.Dims3.z);
      Done = true;
    }
  }

  /* recurse */
  tree* Left = nullptr; 
  if (Begin+1==Mid && CellCountLeft==1) {
    assert(Depth+1 == Params.MaxDepth);
    Left = new (TreePtr++) tree;
    Left->Count = 1;
    ++NParticlesDecoded;
    //ParticleLevels[ResLvl+(Split==ResolutionSplit)].push_back(particle_int{.Pos = (Particles[Begin].Pos - GridLeft.From3) / GridLeft.Stride3});
  } else if (Begin < Mid) { // recurse
    assert(Depth+1 < Params.MaxDepth);
    //split_type NextSplit = 
    //  ((Depth+1==Params.StartResolutionSplit) ||
    //   (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;
    //if (Split == SpatialSplit)
    //  Left = BuildTreeIntPredict(PredNode?PredNode->Left:nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);
    //else if (Split == ResolutionSplit)
    //  Left = BuildTreeIntPredict(nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+1, Depth+1);      
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit && ResLvl+2<Params.NLevels) ? ResolutionSplit : SpatialSplit;
    Left = BuildTreeIntPredict(nullptr, Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);      
  }

  /* recurse on the right */
  tree* Right = nullptr;
  if (Mid+1==End && CellCountRight==1) {
    assert(Depth+1 == Params.MaxDepth);
    Right = new (TreePtr++) tree;
    Right->Count = 1;
    ++NParticlesDecoded;
    //ParticleLevels[ResLvl+(Split==ResolutionSplit)].push_back(particle_int{.Pos = (Particles[Mid].Pos - GridRight.From3) / GridRight.Stride3});
  } else if (Mid < End) { //recurse
    assert(Depth+1 < Params.MaxDepth);
    //split_type NextSplit = SpatialSplit;
    //if (Split == SpatialSplit)
    //  Right = BuildTreeIntPredict(PredNode?PredNode->Right:nullptr, Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    //else if (Split == ResolutionSplit)
    //  Right = BuildTreeIntPredict(Left, Particles, Mid, End, GridRight, NextSplit, ResLvl, Depth+1);
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit && ResLvl+2<Params.NLevels) ? ResolutionSplit : SpatialSplit;
    Right = BuildTreeIntPredict(nullptr, Particles, Mid, End, GridRight, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);
  }

  //if (Depth < Params.MaxDepth) {
  //  assert(Left || Right);
  //}

  /* construct the prediction tree */
  tree* Node = nullptr;
  //if (Split == ResolutionSplit) {
  //  //printf("Resolution Split\n");
  //  Node = BuildPredTree(Left, Right, Depth, D);
  //} else if (Split == SpatialSplit) {
  //  Node = new (TreePtr++) tree;
  //  Node->Left  = Left;
  //  Node->Right = Right;
  //  if (Left ) Node->Count = Left->Count; else Node->Count = 0;
  //  if (Right) Node->Count += Right->Count;
  //}
  // TODO: deallocate the two merged trees
  /* output the trees to test with mpeg */
  //if (Split == ResolutionSplit) {
  //  for (i64 I = Mid; I < End; ++I) {
  //    Particles[I].Pos = (Particles[I].Pos - GridRight.From3) / GridRight.Stride3;
  //  }
  //  WritePLYInt(PRINT("%s-%d.ply", Params.Name, ResLvl), Particles.begin() + Mid, Particles.begin() + End);
  //  split_type NextSplitLeft = 
  //    ((Depth+1==Params.StartResolutionSplit) ||
  //     (Split==ResolutionSplit && ResLvl+2<Params.NLevels)) ? ResolutionSplit : SpatialSplit;

  //  if (NextSplitLeft == SpatialSplit) { // last resolution split
  //    for (i64 I = Begin; I < Mid; ++I) {
  //      Particles[I].Pos = (Particles[I].Pos - GridLeft.From3) / GridLeft.Stride3;
  //    }
  //    WritePLYInt(PRINT("%s-%d.ply", Params.Name, ResLvl+1), Particles.begin() + Begin, Particles.begin() + Mid);
  //  }
  //}

  return Node;
}

static void
BuildTreeIntGeneral(
  std::vector<particle_int>& Particles, i64 Begin, i64 End, const grid_int& Grid, 
  split_type Split, i8 ResLvl, i8 Depth) 
{
  /* early return if the number of particles is the same as the number of cells */
  i64 N = End - Begin; // total number of particles
  i64 CellCount = i64(Grid.Dims3.x) * i64(Grid.Dims3.y) * i64(Grid.Dims3.z);
  //if (CellCount == N) return;

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
  if (CellCount-N < N) {
    N = CellCount - N;
    P = CellCountLeft - P;
  }
  //N = MIN(N, CellCountRight); // this only makes sense if the grid dimension is non power of two (so that the right can have fewer cells than the left)
  EncodeCenteredMinimal(u32(P), u32(N+1), &BlockStream);

  if (Depth+1 == Params.StartResolutionSplit) { // print the size of the grid where the resolution split first happens
    static bool Done = false;
    if (!Done) {
      printf("grid dims is %d %d %d\n", Grid.Dims3.x, Grid.Dims3.y, Grid.Dims3.z);
      Done = true;
    }
  }

  /* recurse */
  tree* Left = nullptr; 
  if (Begin+1==Mid && CellCountLeft==1) {
    ++NParticlesDecoded;
    bbox_int BBox; BBox.Min = GridLeft.From3 * Params.W3; BBox.Max = BBox.Min + Params.W3 - 1;
    for (int DD = 0; DD < 2; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Begin].Pos[D] <= M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        Write(&BlockStream, Left);
      }
    }
    //ParticleLevels[ResLvl+(Split==ResolutionSplit)].push_back(particle_int{.Pos = (Particles[Begin].Pos - GridLeft.From3) / GridLeft.Stride3});
  } else if (Begin < Mid) { // recurse
    assert(Depth+1 < Params.MaxDepth);
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit && ResLvl+2<Params.NLevels) ? ResolutionSplit : SpatialSplit;
    BuildTreeIntGeneral(Particles, Begin, Mid, GridLeft, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);      
  }

  /* recurse on the right */
  tree* Right = nullptr;
  if (Mid+1==End && CellCountRight==1) {
    ++NParticlesDecoded;
    bbox_int BBox; BBox.Min = GridRight.From3 * Params.W3; BBox.Max = BBox.Min + Params.W3 - 1;
    for (int DD = 0; DD < 2; ++DD) {
      while (BBox.Max[DD] > BBox.Min[DD]) {
        i32 M = (BBox.Max[DD]+BBox.Min[DD]) >> 1;
        bool Left = Particles[Begin].Pos[D] <= M;
        if (Left) BBox.Max[DD] = M; else BBox.Min[DD] = M;
        Write(&BlockStream, Left);
      }
    }
  } else if (Mid < End) { //recurse
    assert(Depth+1 < Params.MaxDepth);
    split_type NextSplit = (Depth+1>=Params.StartResolutionSplit && ResLvl+2<Params.NLevels) ? ResolutionSplit : SpatialSplit;
    BuildTreeIntGeneral(Particles, Mid, End, GridRight, NextSplit, ResLvl+(Split==ResolutionSplit), Depth+1);
  }
}

/* Split so that the number of particles are approximately equal on both sides 
(instead of splitting in the middle of the bounding box) */
static void
BuildTreeIntBalance(std::vector<particle_int>& ParticlesInt, i64 Begin, i64 End, bbox_int BBox, split_type Split) {
  auto G = BBox.Max - BBox.Min + 1; // grid formed by BBox
  i8 D = 0;
  if (G[1] > G[D])
    D = 1;
  if (G[2] > G[D])
    D = 2;

  i64 Mid = Begin;
  i32 MM = BBox.Min[D];
  if (Split == BalanceSplit) { // balance split
    auto Pred = [D](const particle_int& P1, const particle_int& P2) {
      return P1.Pos[D] < P2.Pos[D];
    };
    std::sort(ParticlesInt.begin() + Begin, ParticlesInt.begin() + End, Pred);
    Mid = (Begin + End + 1) >> 1;
  } else { // spatial split
    MM = (BBox.Min[D] + BBox.Max[D]) >> 1;
    auto SPred = [MM, D](const particle_int& P) {
      return P.Pos[D] <= MM;
    };
    Mid = std::partition(RANGE(ParticlesInt, Begin, End), SPred) - ParticlesInt.begin();
  }
  bool Stop = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  if (Stop)
    return;
    // encode the bin where Particle[Mid].Pos[D] is
  i64 N = End - Begin; // total number of particles
  i64 P = End - Mid; // number of particles on the right
  i64 CellCount = i64(G.x) * i64(G.y) * i64(G.z);
  i64 CellCountRight = 1;
  if (Split == SpatialSplit) {
    auto BBoxCopy = BBox;
    BBoxCopy.Min[D] = MM + 1;
    auto GR = BBoxCopy.Max - BBoxCopy.Min;
    CellCountRight = i64(GR.x) * i64(GR.y) * i64(GR.z);
  } else if (Split == BalanceSplit) {
    auto BBoxCopy = BBox;
    BBoxCopy.Min[D] = ParticlesInt[Mid-1].Pos[D];
    auto GR = BBoxCopy.Max - BBoxCopy.Min;
    CellCountRight = i64(GR.x) * i64(GR.y) * i64(GR.z);
  }

  if (CellCount - N < N) { // more particles than empty cells, encode the empty cells
    N = CellCount - N;
    P = CellCountRight - P;
  }
  if (N == 0) {
    NParticlesDecoded += End - Begin;
    return;
  }

  i32 M = Begin;
  if (Split == BalanceSplit) {
    M = ParticlesInt[Mid-1].Pos[D]; // split into [B - M], [M - E)
    assert(BBox.Min[D] <= M);
    assert(BBox.Max[D] >= M);
    EncodeCenteredMinimal(M - BBox.Min[D], G[D], &BlockStream);
    SeparationCodeLength += log2(G[D]);
  } else if (Split == SpatialSplit) {
    EncodeCenteredMinimal(u32(P), u32(N + 1), &BlockStream);
    SeparationCodeLength += log2(N + 1);
  }
  /* recurse on the left or right */
  if (Begin < Mid) {
    auto BBoxCopy = BBox;
    if (Split == BalanceSplit) {
      BBoxCopy.Max[D] = M;
    } else if (Split == SpatialSplit) {
      BBoxCopy.Max[D] = MM;
    }
    if (Begin + 1 == Mid) { // left leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // write refinement bits
        vec3i P3 = ParticlesInt[Begin].Pos;
        vec3i Pos3;
        FOR(int, I, 0, Params.NDims) {
          while (BBoxCopy.Max[I] - BBoxCopy.Min[I] > 2 * Params.Accuracy) {
            REQUIRE(BBoxCopy.Min[I] <= P3[I]);
            REQUIRE(BBoxCopy.Max[I] >= P3[I]);
            i32 Half = (BBoxCopy.Min[I] + BBoxCopy.Max[I]) >> 1;
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBoxCopy.Max[I] = Half;
            } else {
              BBoxCopy.Min[I] = Half + 1;
            }
          }
          Pos3[I] = (BBoxCopy.Max[I] + BBoxCopy.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];  
          RMSE += Diff * Diff;
        }
        ++NParticlesDecoded;
      }
    } else { // recurse on the left
      split_type NextSplit = BalanceSplit;
      auto GG = BBoxCopy.Max - BBoxCopy.Min + 1; // grid formed by BBox
      i8 DD = 0;
      if (GG[1] > GG[DD])
        DD = 1;
      if (GG[2] > GG[DD])
        DD = 2;
      if (BBoxCopy.Max[DD] - BBoxCopy.Min[DD] < Mid - Begin) { // fewer particles (should we switch to the other mode?)
        NextSplit = SpatialSplit;
      }
      //split_type NextSplit = SPLIT;
      BuildTreeIntBalance(ParticlesInt, Begin, Mid, BBoxCopy, NextSplit);
    }
  }

  if (Mid < End) {
    auto BBoxCopy = BBox;
    if (Split == BalanceSplit) {
      BBoxCopy.Min[D] = M;
    } else if (Split == SpatialSplit) {
      BBoxCopy.Min[D] = MM + 1;
    }
    if (Mid + 1 == End) { // right leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        vec3i Pos3;
        vec3i P3 = ParticlesInt[Mid].Pos;
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          while (BBoxCopy.Max[I] - BBoxCopy.Min[I] > 2 * Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
            REQUIRE(BBoxCopy.Min[I] <= P3[I]);
            REQUIRE(BBoxCopy.Max[I] >= P3[I]);
            i32 Half = (BBoxCopy.Min[I] + BBoxCopy.Max[I]) >> 1;
            bool LeftSide = P3[I] <= Half;
            Write(&BlockStream, LeftSide);
            ++RefinementCodeLength;
            if (LeftSide) {
              BBoxCopy.Max[I] = Half;
            } else {
              BBoxCopy.Min[I] = Half + 1;
            }
          }
          Pos3[I] = (BBoxCopy.Max[I] + BBoxCopy.Min[I]) >> 1;
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
        }
        ++NParticlesDecoded;
      }
    } else { // recurse on the right
      split_type NextSplit = BalanceSplit;
      auto GG = BBoxCopy.Max - BBoxCopy.Min + 1; // grid formed by BBox
      i8 DD = 0;
      if (GG[1] > GG[DD])
        DD = 1;
      if (GG[2] > GG[DD])
        DD = 2;
      if (BBoxCopy.Max[DD] - BBoxCopy.Min[DD] < End - Mid) { // fewer particles (should we switch to the other mode?)
        NextSplit = SpatialSplit;
      }
      //split_type NextSplit = SPLIT;
      BuildTreeIntBalance(ParticlesInt, Mid, End, BBoxCopy, NextSplit);
    }
  }
}

static void
BuildTreeDFS(i64 Begin, i64 End, u64 Code, const grid& Grid, i8 Level, split_type Split, i8 Depth) {
  i8 D = Params.DimsStr[Depth] - 'x';
  i64 Mid = Begin;
  f32 W = (Params.BBox.Max[D] - Params.BBox.Min[D]) / Params.Dims3[D];
  if (Split == ResolutionSplit) { // resolution split
    auto RPred = [W, D, &Grid](const particle& P) {
      i32 Bin = MIN(Params.Dims3[D] - 1, i32((P.Pos[D] - Params.BBox.Min[D]) / W));
      REQUIRE((Bin - i32(Grid.From3[D])) % i32(Grid.Stride3[D]) == 0);
      Bin = (Bin - i32(Grid.From3[D])) / i32(Grid.Stride3[D]);
      return IS_EVEN(Bin);
    };
    Mid = std::partition(RANGE(Particles, Begin, End), RPred) - Particles.begin();
  } else { // spatial split
    f32 S = (Grid.Dims3[D] > 1.5f) * (Grid.Stride3[D] - 1) + 1; // either Stride or 1
    f32 M = Params.BBox.Min[D] + W * (Grid.From3[D] + Grid.Dims3[D] * 0.5f * S);
    auto SPred = [M, D, &Grid](const particle& P) {
      return P.Pos[D] < M;
    };
    Mid = std::partition(RANGE(Particles, Begin, End), SPred) - Particles.begin();
  }
  vec3f GridDims3((f32)Params.Dims3.x, (f32)Params.Dims3.y, (f32)Params.Dims3.z);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / GridDims3;
  bbox ParentBBox {
    .Min = Params.BBox.Min + Grid.From3 * W3,
    .Max = Params.BBox.Min + (Grid.From3 + (Grid.Dims3 - 1) * Grid.Stride3 + 1) * W3
  };
  bool Stop = Params.RefinementMode == refinement_mode::ERROR_BASED;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > 2 * Params.Accuracy) { // TODO: not enough for lossless
      Stop = false;
      break;
    }
  }
  if (!Stop) {
    //WriteVarByte(&BlockStream, u32(Mid - Begin));
    i64 M = End - Begin;
    i64 N = Mid - Begin;
    i64 CellCount = Grid.Dims3.x * Grid.Dims3.y * Grid.Dims3.z;
    if (CellCount - M < M) {
      //EncodeNode(0, CellCount - M, CellCount / 2 - N);
      if (Split == SpatialSplit)
        EncodeCenteredMinimal(u32(CellCount / 2 - N), u32(CellCount - M + 1), &BlockStream);
      else
        EncodeNode(0, CellCount - M, CellCount / 2 - N);
      REQUIRE(CellCount - M >= CellCount / 2 - N);
      bool EvenCellCount = (CellCount & 1) == 0 || CellCount == 1;
      REQUIRE(EvenCellCount);
    //  //printf("hello\n");
    } else {
      //EncodeNode(0, M, N);
      if (Split == SpatialSplit)
        EncodeCenteredMinimal(u32(N), u32(M + 1), &BlockStream);
      else
        EncodeNode(0, M, N);
    }
  } else {
    return;
  }
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // left leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) { // write refinement bits
        vec3f P3 = Particles[Begin].Pos;
        vec3f Pos3;
        auto G = SplitGrid(Grid, D, Split, side::Left);
        bbox BBox {
          .Min = Params.BBox.Min + G.From3 * W3,
          .Max = Params.BBox.Min + (G.From3 + (G.Dims3 - 1) * G.Stride3 + 1) * W3
        };
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          while (G.Dims3[I] > 1 /*&& BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy*/) { // NOTE: enable this to refine the tree uniformly until the leaf level
            G.Dims3[I] *= 0.5f;
            float Half = Params.BBox.Min[I] + (G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I] + 1) * W3[I];
            bool LeftSide = P3[I] < Half;
            Write(&BlockStream, LeftSide);
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
            REQUIRE(BBox.Min[I] <= P3[I]);
            REQUIRE(BBox.Max[I] >= P3[I]);
          }
          REQUIRE(G.Dims3[I] == 1);
          //BBox.Max[I] = BBox.Min[I] + W3[I];
          auto BBoxCopy = BBox;
          BBox.Max[I] = Params.BBox.Min[I] + (G.From3[I] + 1) * W3[I];
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
        }
        FOR(int, I, 0, Params.NDims) { // write refinement bits
          // NOTE: the last condition is for lossless coding
          //while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy && !(BBox.Max[I] <= P3[I] + 1 && BBox.Min[I] >= P3[I] - 1)) {
          while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy && ceil(BBox.Min[I]) != floor(BBox.Max[I])) { // lossless
          //while (ceil(BBox.Min[I]) != P3[I]) {
            //REQUIRE(BBox.Min[I] <= P3[I]);
            //REQUIRE(BBox.Max[I] >= P3[I]);
            float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
            bool LeftSide = P3[I] < Half;
            Write(&BlockStream, LeftSide);
            G.Dims3[I] *= 0.5f;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
          }
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
          if (BBox.Max[I] <= P3[I] + 1 && BBox.Min[I] >= P3[I] - 1)
            Pos3[I] = ceil(BBox.Min[I]);
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
          ++NParticlesDecoded;
        }
      }
    } else { // recurse on the left
      bool RSplit = Split == ResolutionSplit;
      //split_type NextSplit = (RSplit && Level > 1) ? ResolutionSplit : SpatialSplit;
      split_type NextSplit = Grid.Dims3.x * Grid.Dims3.y * Grid.Dims3.z > THRESHOLD ? SpatialSplit : ResolutionSplit;
      BuildTreeDFS(Begin, Mid, (Code * 2 + 1), SplitGrid(Grid, D, Split, side::Left), 
        Level - RSplit, NextSplit, Depth + 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // right leaf
      if (Params.RefinementMode != refinement_mode::SEPARATION_ONLY) {
        vec3f Pos3;
        vec3f P3 = Particles[Mid].Pos;
        auto G = SplitGrid(Grid, D, Split, side::Right);
        bbox BBox {
          .Min = Params.BBox.Min + G.From3 * W3,
          .Max = Params.BBox.Min + (G.From3 + (G.Dims3 - 1) * G.Stride3 + 1) * W3
        };
        FOR(int, I, 0, Params.NDims) { // go until one particle per cell
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          while (G.Dims3[I] > 1/* && BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy*/) { // NOTE: enable this to refine the tree uniformly until the leaf level
            G.Dims3[I] *= 0.5f;
            float Half = Params.BBox.Min[I] + (G.From3[I] + (G.Dims3[I] - 1) * G.Stride3[I] + 1) * W3[I];
            bool LeftSide = P3[I] < Half;
            Write(&BlockStream, LeftSide);
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
            REQUIRE(BBox.Min[I] <= P3[I]);
            REQUIRE(BBox.Max[I] >= P3[I]);
          }
          REQUIRE(G.Dims3[I] == 1);
          //BBox.Max[I] = BBox.Min[I] + W3[I];
          BBox.Max[I] = Params.BBox.Min[I] + (G.From3[I] + 1) * W3[I];
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
        }
        FOR(int, I, 0, Params.NDims) { // write the refinement bits
          // NOTE: the last condition is for lossless coding
          //while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy && !(BBox.Max[I] <= P3[I] + 1 && BBox.Min[I] >= P3[I] - 1)) {
          while (BBox.Max[I] - BBox.Min[I] > 2 * Params.Accuracy && ceil(BBox.Min[I]) != floor(BBox.Max[I])) { // lossless
          //while (ceil(BBox.Min[I]) != P3[I]) {
            float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
            bool LeftSide = P3[I] < Half;
            Write(&BlockStream, LeftSide);
            G.Dims3[I] *= 0.5f;
            if (LeftSide) {
              BBox.Max[I] = Half;
            } else {
              G.From3[I] += G.Dims3[I] * G.Stride3[I];
              BBox.Min[I] = Half;
            }
          }
          REQUIRE(BBox.Min[I] <= P3[I]);
          REQUIRE(BBox.Max[I] >= P3[I]);
          Pos3[I] = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
          if (BBox.Max[I] <= P3[I] + 1 && BBox.Min[I] >= P3[I] - 1)
            Pos3[I] = ceil(BBox.Min[I]);
          auto Diff = Pos3[I] - P3[I];
          RMSE += Diff * Diff;
          ++NParticlesDecoded;
        }
      }
    } else { // recurse on the right
      //BuildTreeDFS(Mid, End, (Code * 2 + 2), SplitGrid(Grid, D, Split, Right), 
      //  Level, SpatialSplit, Depth + 1);
      split_type NextSplit = Grid.Dims3.x * Grid.Dims3.y * Grid.Dims3.z > THRESHOLD ? SpatialSplit : ResolutionSplit;
      BuildTreeDFS(Mid, End, (Code * 2 + 2), SplitGrid(Grid, D, Split, side::Right), 
        Level, NextSplit, Depth + 1);
    }
  }
}

/* we divide the particles into three trees, one for each x/y/z splits 
(so that the particles form a linear order) */
// first x, second x, third x
// 

static i64 BitsSkipped = 0;

static void
DecompressBlockZfp(f64 Accuracy, i32 N, f64* BlockFloats, bitstream* Stream) {
  buffer_t BufFloats(BlockFloats, 64);
  buffer_t BufInts((i64*)BlockFloats, 64);
  u64 BlockUInts[4 * 4 * 4] = {};
  buffer_t BufUInts (BlockUInts, 64);
  i16 EMax = (i16)Read(Stream, traits<f32>::ExpBits) - traits<f32>::ExpBias;
  const i8 NBitPlanes = BIT_SIZE_OF(u64);
  i8 M = 0;
  for (int Bp = NBitPlanes - 1; Bp >= 0; --Bp) {
    bool TooHighPrecision = 5 - NBitPlanes + Bp < Exponent(Accuracy) - EMax;
    if (TooHighPrecision) break;
    Decode(BlockUInts, 64, Bp, M, Stream);
  }
  InverseShuffle(BlockUInts, (i64*)BlockFloats, 3);
  InverseZfp((i64*)BlockFloats, 3);
  const int Prec = NBitPlanes - 1 - 3;
  Dequantize(EMax, Prec, BufInts, &BufFloats);
}

static void
CompressBlockZfp(f64 Accuracy, i32 N, f64* BlockFloats, bitstream* Stream) {
  /* extrapolate to 4x4x4 block */
  i32 S = N % (4 * 4), Z = N / (4 * 4); // z slice
  i32 X = S % 4, Y = S / 4;
  if (X > 0) 
    PadBlock1D(BlockFloats + Z * (4 * 4) + Y * 4, X, 1); // pad line
  if (S > 0 && Y < 3)
    PadBlock2D(BlockFloats + Z * (4 * 4), vec2i(4, Y + 1)); // pad slice
  if (Z < 3)
    PadBlock3D(BlockFloats, vec3i(4, 4, Z + 1));
  /* compress */
  buffer_t BufFloats(BlockFloats, 64);
  buffer_t BufInts((i64*)BlockFloats, 64);
  u64 BlockUInts[4 * 4 * 4];
  buffer_t BufUInts (BlockUInts, 64);
  const i8 NBitPlanes = BIT_SIZE_OF(u64);
  const i8 Prec = NBitPlanes - 1 - 3; // 3 is the number of dimensions
  const i16 EMax = (i16)QuantizeF32(Prec, BufFloats, &BufInts);

  Write(Stream, EMax + traits<f32>::ExpBias, traits<f32>::ExpBits);
  ForwardZfp((i64*)BlockFloats, 3); // 3 = number of dimensions
  ForwardShuffle((i64*)BlockFloats, BlockUInts, 3); // 3 = number of dimensions
  GrowIfTooFull(&BlockStream);
  i8 M = 0;
  for (int Bp = NBitPlanes - 1; Bp >= 0; --Bp) {
    bool TooHighPrecision = 5 - NBitPlanes + Bp < Exponent(Accuracy) - EMax;
    if (TooHighPrecision) break;
    Encode(BlockUInts, 64, Bp, M, Stream);
  }
}

static void
NaiveCompressUsingZfp(f64 Accuracy) {
  std::vector<f64> AllVals(Particles.size());
  f64 BlockVals[64] = {};
  FOR(i32, D, 0, 3) {
    FOR(i64, I, 0, AllVals.size()) {
      AllVals[I] = Particles[I].Pos[D];
    }
    std::sort(AllVals.begin(), AllVals.end());
    for (i64 I = 0; I < AllVals.size(); I += 64) {
      i32 J = I;
      for (; J < I + 64 && J < AllVals.size(); ++J) {
        BlockVals[J - I] = AllVals[J];
      }
      CompressBlockZfp(Accuracy, J - I, BlockVals, &BlockStream);
    }
  }
}

/* Compress one block with zfp */
static void
DecompressBlock() {
  printf("decompressing blocks\n");
  vec3i B3 = Params.BlockDims3;
  vec3f GridDims3((f32)Params.Dims3.x, (f32)Params.Dims3.y, (f32)Params.Dims3.z);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / GridDims3;
  auto ParticlesCopy = Particles;
  if (Params.NDims >= 3) {
    /* handle the Z dimension */
    printf("processing Z -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.z);
    FOR(i32, Z, 0, B3.z) {
      ProjectedCells[Z].reserve(128);
      FOR(i32, Y, 0, B3.y) FOR(i32, X, 0, B3.x) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 4) != 0) {
          ProjectedCells[Z].push_back(C);
        }
      }
    }
    i32 N = 1;
    i32 LastColumn = 64;
    while (N) {
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 z = 0;
      FOR(i32, Z, 0, LastColumn) {
        if (Z >= B3.z) break;
        if (ProjectedCells[Z].empty()) goto OUT_Z;
        ProjCells[z++] = ProjectedCells[Z].back();
        ProjectedCells[Z].pop_back();
        ++N;
      OUT_Z:
        if (Z + 1 == LastColumn && LastColumn < B3.z && z < 64)
          ++LastColumn;
      }
      /* decompress */
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          Particles[C.ParticleId].Pos.z = BlockFloats[I] + I * 1.f * W3.z;
          auto Diff = ParticlesCopy[C.ParticleId].Pos.z - Particles[C.ParticleId].Pos.z;
          RMSE += Diff * Diff;
        }
      }
    }
  }

  /* process Y */
  if (Params.NDims >= 2) {
    printf("processing Y -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.y);
    FOR(i32, Y, 0, B3.y) {
      ProjectedCells[Y].reserve(128);
      FOR(i32, Z, 0, B3.z) FOR(i32, X, 0, B3.x) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 2) != 0)
          ProjectedCells[Y].push_back(C);
      }
    }
    i32 N = 1; // number of particles processed in this "layer"
    i32 LastColumn = 64;
    while (N) { // while there are still particles unprocessed
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 y = 0;
      FOR(i32, Y, 0, LastColumn) {
        if (Y >= B3.y) break;
        if (ProjectedCells[Y].empty()) goto OUT_Y;
        ProjCells[y++] = ProjectedCells[Y].back();
        ProjectedCells[Y].pop_back();
        ++N;
      OUT_Y:
        if (Y + 1 == LastColumn && LastColumn < B3.y && y < 64)
          ++LastColumn;
      }
      /* decompress */
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          Particles[C.ParticleId].Pos.y = BlockFloats[I] + I * 1.f * W3.y;
          auto Diff = ParticlesCopy[C.ParticleId].Pos.y - Particles[C.ParticleId].Pos.y;
          RMSE += Diff * Diff;
        }
      }
    }
  }

  /* process X */
  if (Params.NDims >= 1) {
    printf("processing X -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.x);
    FOR(i32, X, 0, B3.x) {
      ProjectedCells[X].reserve(128);
      FOR(i32, Z, 0, B3.z) FOR(i32, Y, 0, B3.y) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 1) != 0)
          ProjectedCells[X].push_back(C);
      }
    }
    i32 N = 1; // number of particles processed in this "layer"
    i32 LastColumn = 64;
    while (N) { // while there are still particles unprocessed
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 x = 0;
      FOR(i32, X, 0, LastColumn) {
        if (X >= B3.x) break;
        if (ProjectedCells[X].empty()) goto OUT_X;
        ProjCells[x++] = ProjectedCells[X].back();
        ProjectedCells[X].pop_back();
        ++N;
      OUT_X:
        if (X + 1 == LastColumn && LastColumn < B3.x && x < 64)
          ++LastColumn;
      }
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          Particles[C.ParticleId].Pos.x = BlockFloats[I] + I * 1.f * W3.x;
          auto Diff = ParticlesCopy[C.ParticleId].Pos.x - Particles[C.ParticleId].Pos.x;
          RMSE += Diff * Diff;
        }
      }
    }
  }
}
/* Compress one block with zfp */
static void
CompressBlock() {
  vec3i B3 = Params.BlockDims3;
  vec3f GridDims3((f32)Params.Dims3.x, (f32)Params.Dims3.y, (f32)Params.Dims3.z);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / GridDims3;
  bitstream Bs;
  InitWrite(&Bs, 10000000);
  if (Params.NDims >= 3) {
    /* handle the Z dimension */
    printf("processing Z -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.z);
    i32 T = 0;
    FOR(i32, Z, 0, B3.z) {
      ProjectedCells[Z].reserve(128);
      FOR(i32, Y, 0, B3.y) FOR(i32, X, 0, B3.x) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 4) != 0) {
          ProjectedCells[Z].push_back(C);
        }
      }
      T += (i32)ProjectedCells[Z].size();
    }
    //if (T != Params.NParticles)
    //  return; // the tree is not built to the end, no need to do zfp compression
    //assert(T == Params.NParticles);
    i32 Count[65] = {};
    i32 N = 1; // number of particles processed in this "layer"
    i32 Avg = 0;
    i32 C = 0;
    i32 LastColumn = 64;
    while (N) { // while there are still particles unprocessed
      ++C;
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 z = 0;
      if (LastColumn == 255)
        int Stop = 0;
      FOR(i32, Z, 0, LastColumn) {
        if (Z >= B3.z) break;
        if (ProjectedCells[Z].empty()) goto OUT_Z;
        ProjCells[z++] = ProjectedCells[Z].back();
        ProjectedCells[Z].pop_back();
        ++Avg;
        ++N;
      OUT_Z:
        if (Z + 1 == LastColumn && LastColumn < B3.z && z < 64)
          ++LastColumn;
      }
      ++Count[N];
      /* compress */
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        f64 BlockFloatsCopy[4 * 4 * 4];
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          BlockFloats[I] = BlockFloatsCopy[I] = Particles[C.ParticleId].Pos.z - I * 1.f * W3.z;
        }
        //CompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        InitWrite(&Bs, Bs.Stream);
        CompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        Flush(&Bs);
        StreamSize += BitSize(Bs);
        InitRead(&Bs, Bs.Stream);
        FOR(i32, I, 0, 64) {
          BlockFloats[I] = 0;
        }
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        FOR(i32, I, 0, N) {
          auto Diff = BlockFloats[I] - BlockFloatsCopy[I];
          RMSE += Diff * Diff;
        }
      }
    }
    Avg /= C;
    printf("   Avg N = %d\n", Avg);
  }

  /* process Y */
  if (Params.NDims >= 2) {
    printf("processing Y -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.y);
    FOR(i32, Y, 0, B3.y) {
      ProjectedCells[Y].reserve(128);
      FOR(i32, Z, 0, B3.z) FOR(i32, X, 0, B3.x) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 2) != 0)
          ProjectedCells[Y].push_back(C);
      }
    }
    i32 N = 1; // number of particles processed in this "layer"
    i32 Avg = 0;
    i32 C = 0;
    i32 LastColumn = 64;
    while (N) { // while there are still particles unprocessed
      ++C;
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 y = 0;
      FOR(i32, Y, 0, LastColumn) {
        if (Y >= B3.y) break;
        if (ProjectedCells[Y].empty()) goto OUT_Y;
        ProjCells[y++] = ProjectedCells[Y].back();
        ProjectedCells[Y].pop_back();
        ++N;
      OUT_Y:
        if (Y + 1 == LastColumn && LastColumn < B3.y && y < 64)
          ++LastColumn;
      }
      Avg += N;
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        f64 BlockFloatsCopy[4 * 4 * 4];
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          BlockFloats[I] = BlockFloatsCopy[I] = Particles[C.ParticleId].Pos.y - I * 1.f * W3.y;
        }
        //CompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        InitWrite(&Bs, Bs.Stream);
        CompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        Flush(&Bs);
        StreamSize += BitSize(Bs);
        InitRead(&Bs, Bs.Stream);
        FOR(i32, I, 0, 64) {
          BlockFloats[I] = 0;
        }
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        FOR(i32, I, 0, N) {
          auto Diff = BlockFloats[I] - BlockFloatsCopy[I];
          RMSE += Diff * Diff;
        }
      }
    }
    Avg /= C;
    printf("   Avg N = %d\n", Avg);
  }

  ///* process X */
  if (Params.NDims >= 1) {
    printf("processing X -------------\n");
    std::vector<std::vector<particle_cell>> ProjectedCells(B3.x);
    FOR(i32, X, 0, B3.x) {
      ProjectedCells[X].reserve(128);
      FOR(i32, Z, 0, B3.z) FOR(i32, Y, 0, B3.y) {
        const auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId >= 0 && (C.DimsEncode & 1) != 0)
          ProjectedCells[X].push_back(C);
      }
    }
    i32 N = 1; // number of particles processed in this "layer"
    i32 Avg = 0;
    i32 C = 0;
    i32 LastColumn = 64;
    while (N) { // while there are still particles unprocessed
      ++C;
      N = 0;
      particle_cell ProjCells[64] = {};
      i32 x = 0;
      FOR(i32, X, 0, LastColumn) {
        if (X >= B3.x) break;
        if (ProjectedCells[X].empty()) goto OUT_X;
        ProjCells[x++] = ProjectedCells[X].back();
        ProjectedCells[X].pop_back();
        ++N;
      OUT_X:
        if (X + 1 == LastColumn && LastColumn < B3.x && x < 64)
          ++LastColumn;
      }
      Avg += N;
      if (N > 0) {
        f64 BlockFloats[4 * 4 * 4] = {};
        f64 BlockFloatsCopy[4 * 4 * 4];
        FOR(int, I, 0, N) { // copy the data over
          const auto& C = ProjCells[I];
          BlockFloats[I] = BlockFloatsCopy[I] = Particles[C.ParticleId].Pos.x - I * 1.f * W3.x;
        }
        //CompressBlockZfp(Params.Accuracy, N, BlockFloats, &BlockStream);
        InitWrite(&Bs, Bs.Stream);
        CompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        Flush(&Bs);
        StreamSize += BitSize(Bs);
        InitRead(&Bs, Bs.Stream);
        FOR(i32, I, 0, 64) {
          BlockFloats[I] = 0;
        }
        DecompressBlockZfp(Params.Accuracy, N, BlockFloats, &Bs);
        FOR(i32, I, 0, N) {
          auto Diff = BlockFloats[I] - BlockFloatsCopy[I];
          RMSE += Diff * Diff;
        }
      }
    }
    Avg /= C;
    printf("   Avg N = %d\n", Avg);
  }
  Dealloc(&Bs);
}

static void
CompressBlockFast() {
  vec3i B3 = Params.BlockDims3;
  if (Params.NDims == 3) {
    /* handle the Z dimension */
    printf("processing Z -------------\n");
    i32 Count[65] = {};
    i32 Avg = 0;
    i32 N = 1; // number of particles processed in this "layer"
    particle_cell ProjCells[64] = {};
    /* project along z */
    FOR(i32, Y, 0, B3.y) FOR(i32, X, 0, B3.x) {
      N = 0;
      i32 z = 0;
      FOR(i32, Z, 0, B3.z) {
        auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId < 0) continue;
        ProjCells[z++] = C;
        ++N;
      }
      Avg += N;
    }
    Avg /= (B3.x * B3.y);
    printf("   Avg N = %d\n", Avg);
  }

  /* process Y */
  if (Params.NDims >= 2) {
    printf("processing Y -------------\n");
    i32 Count[65] = {};
    i32 N = 1; // number of particles processed in this "layer"
    i32 Avg = 0;
    particle_cell ProjCells[64] = {};
    /* project along y */
    FOR(i32, Z, 0, B3.z) FOR (i32, X, 0, B3.x) {
      N = 0;
      i32 y = 0;
      FOR(i32, Y, 0, B3.y) {
        auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId < 0) continue;
        ProjCells[y++] = C;
        ++N;
      }
      Avg += N;
    }
    Avg /= (B3.z * B3.x);
    printf("   Avg N = %d\n", Avg);
  }

  /* process X */
  if (Params.NDims >= 1) {
    printf("processing X -------------\n");
    i32 Count[65] = {};
    i32 Avg = 0;
    i32 N = 1; // number of particles processed in this "layer"
    particle_cell ProjCells[64] = {};
    /* project along x */
    FOR(i32, Z, 0, B3.z) FOR (i32, Y, 0, B3.y) {
      N = 0;
      i32 x = 0;
      FOR(i32, X, 0, B3.x) {
        auto& C = ParticleCells[ROW3(B3.x, B3.y, B3.z, X, Y, Z)];
        if (C.ParticleId < 0) continue;
        ProjCells[x++] = C;
        ++N;
      }
      Avg += N;
    }
    Avg /= (B3.z * B3.y);
    printf("   Avg N = %d\n", Avg);
      // TODO: compress along Y
  }
}

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
  for (i64 I = 1; I < Input.size(); ++I) {
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
  printf("N = %d %d %d %lld\n", Dims3.x, Dims3.y, Dims3.z, N);
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
  printf("dimx\n");
  while (NewDims3.x < Dims3.x) { NewDims3.x <<= 1; printf("%d\n", NewDims3.x); }
  printf("dimy\n");
  while (NewDims3.y < Dims3.y) { NewDims3.y <<= 1; printf("%d\n", NewDims3.y); }
  printf("dimz\n");
  while (NewDims3.z < Dims3.z) { NewDims3.z <<= 1; printf("%d\n", NewDims3.z); }
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

int
main(int Argc, cstr* Argv) {
  //ProcessSemantic3D("D:/Downloads/sg27_station8_intensity_rgb.txt", "D:/Downloads/sg27_station8_intensity_rgb.vtu");
  //return 0;
  //{
  //  i32 Prob = 0;
  //  ToInt(Argv[1], &Prob);
  //  auto Particles = GenerateRandomParticles(Prob);
  //  WritePLYInt(PRINT("%random-%d.ply", Prob), Particles);
  //  return 0;
  //}
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
    if (!OptVal(Argc, Argv, "--height", &Params.MaxHeight)) {
      if (!OptVal(Argc, Argv, "--accuracy", &Params.Accuracy))
        EXIT_ERROR("missing --height and --accuracy");
    }
    //Params.NoRefinement = OptExists(Argc, Argv, "--no_refinement");
    char Temp[32];
    cstr Str = Temp;
    OptVal(Argc, Argv, "--refinement", &Str);
    if (strcmp(Str, "error"     ) == 0) Params.RefinementMode = refinement_mode::ERROR_BASED;
    if (strcmp(Str, "lossless"  ) == 0) Params.RefinementMode = refinement_mode::LOSSLESS;
    if (strcmp(Str, "separation") == 0) Params.RefinementMode = refinement_mode::SEPARATION_ONLY;
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
//    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    //if (!OptVal(Argc, Argv, "--block", &Params.BlockBits)) EXIT_ERROR("missing --block");
    ParticlesInt = ReadParticlesInt(Params.InFile);
    //ParticlesInt = RemoveRepeatedParticles(ParticlesInt);
    if (ParticlesInt.size() == 0)
      EXIT_ERROR("No particles read");
    Params.NParticles = ParticlesInt.size();
    printf("number of particles = %zu\n", ParticlesInt.size());
    double start_time = timer();
    Params.BBoxInt = ComputeBoundingBox(ParticlesInt);
    printf("bbox = (%d %d %d) - (%d %d %d)\n", 
      Params.BBoxInt.Min[0], Params.BBoxInt.Min[1], Params.BBoxInt.Min[2],
      Params.BBoxInt.Max[0], Params.BBoxInt.Max[1], Params.BBoxInt.Max[2]);
    //Params.BaseHeight = Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z;
    //printf("log dims 3 = " PRIvec3i "\n", EXPvec3(Params.LogDims3));
    //BlockStreams.resize(Params.NLevels + 1);
    //EncodeRootNew(Particles.size());
    /* compute the maximum height based on the accuracy */
    //if (Params.MaxHeight == 255) {
    //  Params.MaxHeight = 0;
    //  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
    //  while (W3.x > Params.Accuracy) { ++Params.MaxHeight; W3.x *= 0.5; }
    //  while (W3.y > Params.Accuracy) { ++Params.MaxHeight; W3.y *= 0.5; }
    //  while (W3.z > Params.Accuracy) { ++Params.MaxHeight; W3.z *= 0.5; }
    //}
    //Params.MaxHeight = MAX(Params.MaxHeight, Params.BaseHeight);
    //Params.BlockDims3 = Params.Dims3;
    //BuildTreeNew(q_item_new{ .Begin = 0,
    //                          .End = (i64)Particles.size(),
    //                          .Idx = 1,
    //                          .Grid = Grid,
    //                          .Height = 0 }, Params.Accuracy);

    ////ParticleCells.resize(PROD(Params.BlockDims3));
    CdfTable = CreateBinomialTable(BinomialCutoff);
    //BinomialTables = CreateGeneralBinomialTables();
    //BinomialTablesF64 = CreateGeneralBinomialTablesF64();
    InitWrite(&BlockStream, 900 << 20); // 900 MB
    Coder.InitWrite(900 << 20);
    //Rans64EncInit(&Rans);
    //static size_t OutMaxSize = 32 << 20; // 32 MB
    //static size_t OutMaxElems = OutMaxSize / sizeof(u32);
    //u32* OutBuf = new u32[OutMaxElems]; // TODO: delete this
    //u32* OutEnd = OutBuf + OutMaxElems;
    //RansPtr = OutEnd;
    WriteVarByte(&BlockStream, ParticlesInt.size());
    // TODO: preprocess this
    Params.Dims3 = Params.BBoxInt.Max - Params.BBoxInt.Min + 1; //vec3i(1 << Params.LogDims3.x, 1 << Params.LogDims3.y, 1 << Params.LogDims3.z);
    printf("dims = %d %d %d\n", Params.Dims3[0], Params.Dims3[1], Params.Dims3[2]);
    Params.Dims3 = EnlargeToPow2(Params.Dims3);
    Params.BBoxInt.Max = Params.BBoxInt.Min + Params.Dims3 - 1;
    printf("enlarged dims = %d %d %d\n", Params.Dims3[0], Params.Dims3[1], Params.Dims3[2]);
    Params.MaxDepth = ComputeMaxDepth(Params.Dims3);
    //ComputeGrid(Params.Dims3, Params.DimsStr);
    Params.LogDims3 = ComputeGrid(&ParticlesInt, Params.BBoxInt, 0, ParticlesInt.size(), 0, Params.DimsStr);
    Params.W3[0] = Params.Dims3[0] / (1<<Params.LogDims3[0]);
    Params.W3[1] = Params.Dims3[1] / (1<<Params.LogDims3[1]);
    Params.W3[2] = Params.Dims3[2] / (1<<Params.LogDims3[2]);
    printf("log dims = %d %d %d\n", Params.LogDims3[0], Params.LogDims3[1], Params.LogDims3[2]);
    printf("w3 = %d %d %d\n", Params.W3[0], Params.W3[1], Params.W3[2]);
    grid_int Grid{.From3 = vec3i(0), .Dims3 = Params.Dims3, .Stride3 = vec3i(1)};
    printf("bounding box = (" PRIvec3i ") - (" PRIvec3i ")\n", EXPvec3(Params.BBoxInt.Min), EXPvec3(Params.BBoxInt.Max));
    //BuildTreeIntBalance(ParticlesInt, 0, ParticlesInt.size(), Params.BBoxInt, SPLIT);
    //BuildTreeIntTryBoth(ParticlesInt, 0, ParticlesInt.size(), Params.BBoxInt, 0);
    //BuildTreeInt(ParticlesInt, 0, ParticlesInt.size(), Grid, SPLIT, 0);
    //BuildTreeIntPass1(ParticlesInt, 0, ParticlesInt.size(), Grid, SPLIT, 0);
    //CreateProbTable();
    //BuildTreeIntPass2(ParticlesInt, 0, ParticlesInt.size(), Grid, SPLIT, 0);
    split_type Split = SpatialSplit;
    if (Params.NLevels > 1 && Params.StartResolutionSplit == 0)
      Split = ResolutionSplit;
    ParticleLevels.resize(Params.MaxDepth+1);
    BuildTreeIntGeneral(ParticlesInt, 0, ParticlesInt.size(), Grid, Split, 0, 0);
    //DumpTree(MyNode, true);
    Coder.EncodeFinalize();
    Flush(&BlockStream);
    for (int I = 0; I < ParticleLevels.size(); ++I) {
      if (!ParticleLevels[I].empty())
        WritePLYInt(PRINT("%s-%d.ply", Params.Name, I), ParticleLevels[I].begin(), ParticleLevels[I].end());
    }
    //for (i64 I = 0; I < Residuals.size(); ++I) {
    //  printf("%d\n", Residuals[I]);
    //}
    //bitstream Bs;
    //InitWrite(&Bs, 100 << 20); // 100 MB
    //for (int I = 0; I < Residuals.size(); ++I) {
    //  WriteVarByte(&Bs, Residuals[I]);
    //}
    //FILE* Rf = fopen("residuals.dat", "wb");
    //fwrite(Bs.Stream.Data, Size(Bs), 1, Rf);
    //fclose(Rf);

    //auto ResidualTree = BuildBinaryTreeForResiduals(Residuals);
    //i64 ResidualCodeLength = CountCodeLength(ResidualTree);
    //printf("Residual code length        = %lld\n", ResidualCodeLength);
    printf("Residual code length normal = %lld\n", i64((ResidualCodeLengthNormal+7)/8));
    printf("Residual code length gamma  = %lld\n", i64((ResidualCodeLengthGamma+7)/8));
    double dec_time = timer() - start_time;
    printf("Time: %f s\n", dec_time);
    //Rans64EncFlush(&Rans, &RansPtr);
    //printf("RANS stream size = %d bytes\n", int(OutEnd - RansPtr) * sizeof(u32));
    WriteMetaFile(Params, PRINT("%s.idx", Params.OutFile));
    printf("%s\n", Params.DimsStr);
    i64 BlockStreamSize = Size(BlockStream) + Size(Coder.BitStream);
    FILE* Fp = fopen(PRINT("%s.bin", Params.OutFile), "wb");
    i64 FirstStreamSize = Size(BlockStream);
    //i64 SecondStreamSize = (OutEnd - RansPtr) * sizeof(u32);
    i64 SecondStreamSize = Size(Coder.BitStream);
    fwrite(BlockStream.Stream.Data, FirstStreamSize, 1, Fp);
    fwrite(Coder.BitStream.Stream.Data, SecondStreamSize, 1, Fp);
    //fwrite(RansPtr, SecondStreamSize, 1, Fp);
    fwrite(&FirstStreamSize, sizeof(FirstStreamSize), 1, Fp);
    //i64 SecondStreamSize = Size(Coder.BitStream);
    fwrite(&SecondStreamSize, sizeof(SecondStreamSize), 1, Fp);
    //printf("%lld %lld\n", FirstStreamSize, SecondStreamSize);
    fclose(Fp);
    //printf("Uniform code size 1                = %lld\n", (UniformCodeSize1 + 7) / 8);
    printf("Max depth                          = %d\n", Params.MaxDepth);
    printf("Binomial stream size               = %lld\n", (BinomialCodeSize + 7) / 8);
    //printf("Uniform code size 2                = %lld\n", (UniformCodeSize2 + 7) / 8);
    printf("Range code size                    = %f\n",   (RangeCodeSize + 7) / 8);
    printf("Non-predicted code size            = %lld\n", (NonPredictedCodeSize + 7) / 8);
    printf("predicted node count               = %lld\n", PredictedNodeCount);
    printf("non predicted node count           = %lld\n", NonPredictedNodeCount);
    printf("Stream size                        = %lld\n", BlockStreamSize);
    printf("Separation code size (theoretical) = %f\n", (SeparationCodeLength ) / 8);
    printf("Separation code size (actual)      = %lld\n", BlockStreamSize - (RefinementCodeLength + 7 ) / 8);
    printf("Refinement code size               = %lld\n", (RefinementCodeLength + 7 ) / 8);
    printf("RMSE = %f\n", sqrt(RMSE / (ParticlesInt.size() * Params.NDims)));
    printf("# particles = %lld\n", NParticlesDecoded);
    printf("Average ratio = %f Ratio count = %lld\n", Ratio / RatioCount, RatioCount);
    printf("Nodes with more empty cells count = %lld\n", NodesWithMoreEmptyCellsCount);
    printf("Nodes with more particles count = %lld\n", NodesWithMoreParticlesCount);
    /* dump the debug info */
    //FILE* Ff = fopen("debug.dat", "wb");
    //i64 DebugSize = DebugProbs.size();
    //fwrite(&DebugSize, sizeof(DebugSize), 1, Fp);
    //for (i64 I = 0; I < DebugSize; ++I) {
    //  fwrite(&DebugProbs[I], sizeof(DebugProbs[I]), 1, Fp);
    //}
    //fclose(Ff);
  /* ---------------- DECODING ------------------*/
  } else if (Params.Action == action::Decode) { /* decoding */
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    ReadMetaFile(PRINT("%s.idx", Params.InFile));
    u8 MaxHeight = 0;
    f32 Accuracy = 0;
    if (!OptVal(Argc, Argv, "--height", &MaxHeight)) {
      if (OptVal(Argc, Argv, "--accuracy", &Accuracy)) {
        //EXIT_ERROR("missing --height and --accuracy");
        Params.DecodeAccuracy = Accuracy;
      }
    }
    OptVal(Argc, Argv, "--max_level", &Params.MaxLevel);
    OptVal(Argc, Argv, "--max_num_blocks", &Params.MaxNBlocks);
    OptVal(Argc, Argv, "--max_subsampling", &Params.MaxParticleSubSampling);

    printf("%s\n", Params.DimsStr);
    Params.MaxDepth = ComputeMaxDepth(Params.Dims3);
    //BlockOffsets.resize(Params.NLevels);
    //BlockBytes.resize(Params.NLevels + 1);
    //BlockStreams.resize(Params.NLevels + 1);
    //RefBlockStreams.resize(Params.MaxHeight - Params.BaseHeight);
    printf("baseheight = %d maxheight = %d\n", Params.BaseHeight, Params.MaxHeight);
    FILE* Fp = fopen(PRINT("%s.bin", Params.InFile), "rb");
    FSEEK(Fp, 0, SEEK_END);
    i64 FirstStreamSize = 0, SecondStreamSize = 0;
    ReadBackwardPOD(Fp, &SecondStreamSize);
    ReadBackwardPOD(Fp, &FirstStreamSize);
    AllocBuf(&BlockStream.Stream, FirstStreamSize);
    AllocBuf(&Coder.BitStream.Stream, FirstStreamSize);
    FSEEK(Fp, 0, SEEK_SET);
    fread(BlockStream.Stream.Data, FirstStreamSize, 1, Fp);
    fread(Coder.BitStream.Stream.Data, SecondStreamSize, 1, Fp);
    if (Fp) fclose(Fp);
    double start_time = timer();
    uint64_t dec_start_time = __rdtsc();
    BinomialTables = CreateGeneralBinomialTables();
    Coder.InitRead();
    InitRead(&BlockStream, BlockStream.Stream);
    printf("bit stream size = %lld\n", Size(BlockStream.Stream));
    i64 N = ReadVarByte(&BlockStream);
    printf("DecodeAccuracy = %f\n", Params.DecodeAccuracy);
    grid_int Grid{.From3 = vec3i(0), .Dims3 = Params.Dims3, .Stride3 = vec3i(1)};
    printf("bounding box = (" PRIvec3i ") - (" PRIvec3i ")\n", EXPvec3(Params.BBoxInt.Min), EXPvec3(Params.BBoxInt.Max));
    split_type Split = SpatialSplit;
    if (Params.NLevels>1 && Params.StartResolutionSplit==0)
      Split = ResolutionSplit;
    /* read the debug info */
    //FILE* Ff = fopen("debug.dat", "rb");
    //i64 DebugSize = 0;
    //fread(&DebugSize, sizeof(DebugSize), 1, Fp);
    //DebugProbs.resize(DebugSize);
    //for (i64 I = 0; I < DebugSize; ++I) {
    //  fread(&DebugProbs[I], sizeof(DebugProbs[I]), 1, Fp);
    //}
    //fclose(Ff);
    TreePtr = new tree[N * 10];
    auto TreePtrBackup = TreePtr;
    ParticlesInt.reserve(N);
    tree* MyNode = DecodeTreeIntPredict(nullptr, ParticlesInt, 0, N, Grid, Split, 0, 0);
    delete[] TreePtrBackup;
    uint64_t dec_clocks = __rdtsc() - dec_start_time;
    double dec_time = timer() - start_time;
    printf("%lld clocks, %f s\n", dec_clocks, dec_time);
    printf("consumed stream size = %lld\n", Size(BlockStream));
    WritePLYInt(PRINT("%s.ply", Params.OutFile), ParticlesInt.begin(), ParticlesInt.end());
    printf("num particles decoded = %lld\n", NParticlesDecoded);
    printf("num particles generated = %lld\n", NParticlesGenerated);
    //Blocks.resize(Params.NLevels + 1);
    //Blocks[Params.NLevels].resize(1);
    //Heap.insert(block_data{.Level = Params.NLevels, .Height = 0, .BlockId = 0}, block_priority{.Level = Params.NLevels, .BlockId = 0, .Error = 0});
    //bool Continue = true;
    //int NBlocks = 0;
    //while (Continue && /*NBlocks < Params.MaxNBlocks*/BlockBytesRead < Params.MaxNBlocks) {
    //  //Continue = RefineByLevel();
    //  Continue = RefineByError();
    //  ++NBlocks;
    //}
    //if (!Blocks[Params.NLevels].empty()) {
    //  Particles.reserve(Blocks[Params.NLevels][0].Nodes[0]);
    //  i8 D = 0;
    //  grid Grid{.From3 = vec3f(0), .Dims3 = vec3f(Params.Dims3), .Stride3 = vec3f(1)};
    //  i8 Level = Params.NLevels - 1;
    //  u8 Height = 0;
    //  if (Params.NLevels == 1) {
    //    GenerateParticles(tree_node{
    //      .Level = Level,
    //      .Height = 0,
    //      .NodeId = 1,
    //      .Grid = Grid,
    //      .D = 0
    //    });
    //  } else {
    //    while (true) {
    //      if (Level <= Params.MaxLevel && 0 == GenerateParticles(tree_node{
    //        .Level = Level,
    //        .Height = u8(Height + 1),
    //        .NodeId = 1,
    //        .Grid = SplitGrid(Grid, D, ResolutionSplit, Right),
    //        .D = i8((D + 1) % Params.NDims)
    //      })) {
    //        //GenerateParticlesPerNode(Blocks[Params.NLevels][0].Nodes[LEVEL_TO_NODE(Level)], SplitGrid(Grid, D, ResolutionSplit, Right));
    //      }
    //      Grid = SplitGrid(Grid, D, ResolutionSplit, Left);
    //      D = (D + 1) % Params.NDims;
    //      --Level;
    //      ++Height;
    //      if (Level == 0) {
    //        GenerateParticles(tree_node{
    //          .Level = Level,
    //          .Height = Height,
    //          .NodeId = 1,
    //          .Grid = Grid,
    //          .D = D
    //        });
    //        break;
    //      } 
    //    }
    //  }
    //  printf("ncount = %lld %lld\n", NCount, NCount2);
    //  printf("bytes read = %lld\n", BlockBytesRead);
    //  WriteXYZ(Params.OutFile, Particles.begin(), Particles.end());
    //}
  //================= ERROR =========================
  } else if (Params.Action == action::Error) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    if (!OptVal(Argc, Argv, "--dims", &Params.Dims3)) EXIT_ERROR("missing --dims");
    auto Particles1 = ReadParticlesInt(Params.InFile);
    auto Particles2 = ReadParticlesInt(Params.OutFile);
    //f32 Err1 = Error3(Particles1, Particles2, Params.Dims3);
    //f32 Err2 = Error3(Particles2, Particles1, Params.Dims3);
    //printf("error = %f %f %f\n", Err1, Err2, MAX(Err1, Err2));
    if (!CheckSame(Particles1, Particles2))
      printf("not same\n");
    else
      printf("same\n");
  //================= CONVERT =======================
  } else if (Params.Action == action::Convert) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    bool Quantize = OptExists(Argc, Argv, "--quantize");
    f32 MaxAbsX = 0, MaxAbsY = 0, MaxAbsZ = 0;
    Particles = ReadParticles(Params.InFile);
    fprintf(stderr, "Done reading particles\n");
    ParticlesInt.resize(Particles.size());
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
      for (i64 I = 0; I < Particles.size() ; ++I) {
        ParticlesInt[I].Pos.x = i32(ScaleX * Particles[I].Pos.x);
        ParticlesInt[I].Pos.y = i32(ScaleY * Particles[I].Pos.y);
        ParticlesInt[I].Pos.z = i32(ScaleZ * Particles[I].Pos.z);
      }
      fprintf(stderr, "Done quantizing\n");
      ParticlesInt = RemoveRepeatedParticles(ParticlesInt);
      fprintf(stderr, "Writing particles\n");
      WriteParticlesInt(Params.OutFile, ParticlesInt);
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

