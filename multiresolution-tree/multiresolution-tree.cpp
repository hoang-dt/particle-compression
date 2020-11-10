// multiresolution-tree.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

// TODO: better memory allocation (to put the leaves on the same memory block)
// TODO: memory deallocation?
#define ZFP
#define DOCTEST_CONFIG_IMPLEMENT
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#define SEXPR_IMPLEMENTATION
#include "common.h"
#include "zfp.h"
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

static std::vector<vec3f> GridPoints; // stores the grid points that contain the (to be generated) particles


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
        } else if (SExprStringEqual((cstr)Buf.Data, &(LastExpr->s), "height")) {
          REQUIRE(Expr->type == SE_INT);
          Params.MaxHeight = Expr->i;
          printf("Max height = %d\n", Params.MaxHeight);
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
  f32 Rx = f32(rand()) / f32(RAND_MAX);
  f32 Ry = f32(rand()) / f32(RAND_MAX);
  f32 Rz = f32(rand()) / f32(RAND_MAX);
  vec3f P3 = BBox.Min + (BBox.Max - BBox.Min) * vec3f(Rx, Ry, Rz);
//  vec3f P3 = BBox.Min + (BBox.Max - BBox.Min) * 0.5f;
  Particles.push_back(particle{.Pos = P3});
}

static i64 NCount2 = 0;

/* Generate N random particles inside Grid */
static void
GenerateParticlesPerNode(i64 N, const grid& Grid) {
  if (N == 0) return;
  //if (N <= Params.MaxParticleSubSampling)
  //  N = 1;
  //printf("generating %lld particles\n", N);
  assert(Grid.Dims3.x >= 1 && Grid.Dims3.y >= 1 && Grid.Dims3.z >= 1);
  //GridPoints.reserve(N);
  //GridPoints.clear();
  GridPoints.resize(N);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
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
  if (Node.Height < Params.BaseHeight) { // regular node, 2 children
    if (GetNode(Node, &N) && N > 0) {
      REQUIRE(N <= Params.NParticles);
      if (N <= Params.MaxParticleSubSampling) { // return 1 particle for all N particles
//        GenerateParticlesPerNode(1, Node.Grid);
        GenerateParticlesPerNode(N, Node.Grid);
        return N;
      }
      i64 LeftN = GenerateParticles(
        tree_node{
          .Level = Node.Level,
          .Height = u8(Node.Height + 1),
          .NodeId = Node.NodeId * 2,
          .Grid = SplitGrid(Node.Grid, Node.D, SpatialSplit, Left),
          .D = i8((Node.D + 1) % Params.NDims)
        }
      );
      i64 RightN = GenerateParticles(
        tree_node{
          .Level = Node.Level,
          .Height = u8(Node.Height + 1),
          .NodeId = Node.NodeId * 2 + 1,
          .Grid = SplitGrid(Node.Grid, Node.D, SpatialSplit, Right),
          .D = i8((Node.D + 1) % Params.NDims)
        }
      );
      if (LeftN == 0 && RightN == 0) { // generate particles for the parent
        GenerateParticlesPerNode(N, Node.Grid);
        NCount += N;
        REQUIRE(LeftN + RightN <= N);
      } else if (LeftN > 0 && RightN == 0) { // generate particles for the right child
        GenerateParticlesPerNode(N - LeftN, SplitGrid(Node.Grid, Node.D, SpatialSplit, Right));
        REQUIRE(LeftN + RightN <= N);
        NCount += N - LeftN;
      } else if (LeftN == 0 && RightN > 0) { // generate particles for the left child
        GenerateParticlesPerNode(N - RightN, SplitGrid(Node.Grid, Node.D, SpatialSplit, Left));
        REQUIRE(LeftN + RightN <= N);
        NCount += N - RightN;
      } else {
        if (Params.MaxParticleSubSampling <= 1)
          REQUIRE(LeftN + RightN == N);
      }
    } 
  } else if (Node.Height == Params.BaseHeight) { // refinement node, 1 children
//    if (GetNode(Node, &N) && N > 0) {
//      REQUIRE(N == 1);
//      i64 NNodesAtLeaf = NUM_NODES_AT_LEAF(Node.Level);
//      tree_node ChildNode = Node;
//      ChildNode.Height = Node.Height + 1;
//      ChildNode.NodeId = Node.NodeId + NNodesAtLeaf;
//      i8 D = Node.D; // NOTE: we won't be using ChildNode.D
//      vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
//      assert(Node.Grid.Dims3.x == 1 && Node.Grid.Dims3.y == 1 && Node.Grid.Dims3.z == 1);
//      bbox BBox{
//        .Min = Params.BBox.Min + Node.Grid.From3 * W3,
//        .Max = Params.BBox.Min + (Node.Grid.From3 + 1) * W3
//      };
//      u8 Left = 0;
//      while (ChildNode.Height <= Params.MaxHeight && GetRefNode(ChildNode, &Left)) {
//        float Half = (BBox.Max[D] + BBox.Min[D]) * 0.5f;
////        printf("   level %d node %llu bit %d\n", Node.Level, Node.NodeId, Left);
//        if (Left) BBox.Max[D] = Half; else BBox.Min[D] = Half;
//        ChildNode.NodeId += NNodesAtLeaf;
//        ++ChildNode.Height;
//        D = i8((D + 1) % Params.NDims);
//      }
//      GenerateOneParticle(BBox);
//      ++NCount;
//    }
  }
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

// TODO: write a routine to read from disk and reconstruct the tree/particles
// TODO: write a routine to compute the PSNR of positions
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
          .Grid = SplitGrid(Q.Grid, Q.D, Q.SplitType, Left),
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
          .Grid = SplitGrid(Q.Grid, Q.D, Q.SplitType, Right),
          .D = i8((Q.D + 1) % Params.NDims),
          .Level = Q.Level,
          .Height = u8(Q.Height + 1),
          .SplitType = SpatialSplit
        });
      }
    } else if (!Params.NoRefinement) { // Q.Height == Params.BaseHeight
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

/* Return the dimensions of the underlying grid (in terms of power of two) */
static vec3i
ComputeGrid(
  std::vector<particle>* Particles, const bbox& BBox, 
  i64 Begin, i64 End, i8 Depth, str DimsStr)
{
  REQUIRE(Begin < End); // this cannot be a leaf node
  vec3f BBoxExt3 = Extent(BBox);
  i8 D;
  if (BBoxExt3.x > BBoxExt3.y && BBoxExt3.x > BBoxExt3.z)
    D = 0;
  else if (BBoxExt3.y > BBoxExt3.z && BBoxExt3.y > BBoxExt3.x)
    D = 1;
  else if (BBoxExt3.z > BBoxExt3.y && BBoxExt3.z > BBoxExt3.x)
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

static f32
Error(const std::vector<particle>& Particles1, const std::vector<particle>& Particles2, const vec3i& Dims3) {
  bbox BBox = ComputeBoundingBox(Particles1);
  vec3f W3 = (BBox.Max - BBox.Min) / vec3f(Dims3);
  std::vector<vec3f> Grid(Dims3.x * Dims3.y * Dims3.z);
  FOR_EACH(P, Particles1) {
    vec3i Coord{
      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
    Grid[Coord.z * (Dims3.x * Dims3.y) + Coord.y * (Dims3.x) + Coord.x] = P->Pos;
  }
  float Err = 0;
  FOR_EACH(P, Particles2) {
    vec3i Coord{
      MIN(int((P->Pos.x - BBox.Min.x) / W3.x), Dims3.x - 1), 
      MIN(int((P->Pos.y - BBox.Min.y) / W3.y), Dims3.y - 1), 
      MIN(int((P->Pos.z - BBox.Min.z) / W3.z), Dims3.z - 1)};
    vec3f Diff = Grid[Coord.z * (Dims3.x * Dims3.y) + Coord.y * (Dims3.x) + Coord.x] - P->Pos;
    Err += Diff.x * Diff.x + Diff.y * Diff.y + Diff.z * Diff.z;
  }
  Err = std::sqrt(Err) / Particles2.size();
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

static void
TestCompressSeries(f64 Accuracy) {
  /* generate a series of real values based on a grid */
  i32 From = 0;
  i32 Stride = 0.001;
  i32 Dims = 64; // 100 blocks of 64-values
  std::vector<f64> Vals(Dims);
  for (int I = 0; I < Dims; ++I) {
    f64 Low = From + I * Stride;
    f64 High = From + (I + 1) * Stride;
    f64 X = Low + (High - Low) * rand() / f64(RAND_MAX);
    Vals[I] = X;
  }
  bitstream Stream;
  InitWrite(&Stream, 100000);
  f64 BlockFloats[64];
  f64 BlockFloatsCopy[64];
  FOR(i32, I, 0, 64) BlockFloats[I] = BlockFloatsCopy[I] = Vals[I];
  TestCompressBlockZfp(Accuracy, BlockFloats, &Stream);
  Flush(&Stream);
  printf("stream size = %lld\n", BitSize(Stream));
  InitRead(&Stream, Stream.Stream);
  TestDecompressBlockZfp(Accuracy, BlockFloats, &Stream);
  auto Err = RMSError(buffer_t<f64>(BlockFloatsCopy, 64), buffer_t<f64>(BlockFloats, 64));
  //FOR(i32, I, 0, 64) {
  //  printf("%f %f\n", BlockFloatsCopy[I], BlockFloats[I]);
  //}
  printf("rmse = %f\n", Err);
}

f64 RMSE = 0;
i64 StreamSize = 0;

static void
DecodeTreeDFS(i64 Begin, i64 End, u64 Code, const grid& Grid, i8 Level, split_type Split, i8 Depth) {
  i8 D = DimsStr[Depth] - 'x';
  f32 W = (Params.BBox.Max[D] - Params.BBox.Min[D]) / Params.Dims3[D];
  vec3f GridDims3((f32)Params.Dims3.x, (f32)Params.Dims3.y, (f32)Params.Dims3.z);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / GridDims3;
  bbox ParentBBox{
    .Min = Params.BBox.Min + Grid.From3 * W3,
    .Max = Params.BBox.Min + (Grid.From3 + Grid.Dims3) * W3
  };
  bool Stop = true;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  i64 Mid = Begin;
  if (!Stop)
    Mid = DecodeCenteredMinimal(u32(End - Begin + 1), &BlockStream) + Begin;
    //Mid = ReadVarByte(&BlockStream) + Begin;
  else
    return;
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // left leaf
      vec3f P3 = Particles[Begin].Pos;
      auto G = SplitGrid(Grid, D, Split, Left);
      bbox BBox{
        .Min = Params.BBox.Min + G.From3 * W3,
        .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
      };
      /* decode the refinement bits (NOTE: either toggle this or the next block) */
#if !defined(ZFP)
      FOR(int, I, 0, 3) { // read refinement bits
        while (G.Dims3[I] > 1 && BBox.Max[I] - BBox.Min[I] > Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
        //while (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
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
        float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
        auto Diff = Half - Particles[Begin].Pos[I];
        RMSE += Diff * Diff;
      }
#else
      /* put the particle into the grid */
      vec3i Dims3((i32)G.Dims3.x, (i32)G.Dims3.y, (i32)G.Dims3.z);
      vec3i Stride3((i32)G.Stride3.x, (i32)G.Stride3.y, (i32)G.Stride3.z);
      vec3i From3((i32)G.From3.x, (i32)G.From3.y, (i32)G.From3.z);
      From3 = From3 + Dims3 * Stride3 / 2;
      vec3i B3 = Params.Dims3;
      auto* PCell = &ParticleCells[ROW3(B3.x, B3.y, B3.z, From3.x, From3.y, From3.z)];
      PCell->ParticleId = Begin;
      PCell->Grid = G;
      FOR(i32, I, 0, 3) {
        if (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
          PCell->DimsEncode |= (1 << I);
        }
      }
#endif
    } else { // recurse on the left
      bool RSplit = Split == ResolutionSplit;
      split_type NextSplit = (RSplit && Level > 1) ? ResolutionSplit : SpatialSplit;
      DecodeTreeDFS(Begin, Mid, (Code * 2 + 1), SplitGrid(Grid, D, Split, Left), 
        Level - RSplit, NextSplit, Depth + 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // right leaf
      vec3f P3 = Particles[Mid].Pos;
      auto G = SplitGrid(Grid, D, Split, Right);
      bbox BBox{
        .Min = Params.BBox.Min + G.From3 * W3,
        .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
      };
#if !defined(ZFP)
      /* decode the refinement bits (NOTE: either enable this or the next block) */
      FOR(int, I, 0, 3) {
        while (G.Dims3[I] > 1 && BBox.Max[I] - BBox.Min[I] > Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
        //while (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
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
        float Half = (BBox.Max[I] + BBox.Min[I]) * 0.5f;
        auto Diff = Half - Particles[Mid].Pos[I];
        RMSE += Diff * Diff;
      }
#else
      /* put the particle into the grid */
      vec3i From3((i32)G.From3.x, (i32)G.From3.y, (i32)G.From3.z);
      vec3i Dims3((i32)G.Dims3.x, (i32)G.Dims3.y, (i32)G.Dims3.z);
      vec3i Stride3((i32)G.Stride3.x, (i32)G.Stride3.y, (i32)G.Stride3.z);
      From3 = From3 + Dims3 * Stride3 / 2;
      vec3i B3 = Params.Dims3;
      auto* PCell = &ParticleCells[ROW3(B3.x, B3.y, B3.z, From3.x, From3.y, From3.z)];
      PCell->ParticleId = Mid;
      PCell->Grid = G;
      FOR(i32, I, 0, 3) {
        if (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
          PCell->DimsEncode |= (1 << I);
        }
      }
#endif
    } else { // recurse on the right
      DecodeTreeDFS(Mid, End, (Code * 2 + 2), SplitGrid(Grid, D, Split, Right), 
        Level, SpatialSplit, Depth + 1);
    }
  }
}

static void
BuildTreeDFS(i64 Begin, i64 End, u64 Code, const grid& Grid, i8 Level, split_type Split, i8 Depth) {
  i8 D = DimsStr[Depth] - 'x';
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
  bbox ParentBBox{
    .Min = Params.BBox.Min + Grid.From3 * W3,
    .Max = Params.BBox.Min + (Grid.From3 + Grid.Dims3) * W3
  };
  bool Stop = true;
  FOR(i32, I, 0, 3) {
    if (ParentBBox.Max[I] - ParentBBox.Min[I] > Params.Accuracy) {
      Stop = false;
      break;
    }
  }
  if (!Stop)
    EncodeCenteredMinimal(u32(Mid - Begin), u32(End - Begin + 1), &BlockStream);
    //WriteVarByte(&BlockStream, u32(Mid - Begin));
  else
    return;
  if (Begin < Mid) {
    if (Begin + 1 == Mid) { // left leaf
      vec3f P3 = Particles[Begin].Pos;
      auto G = SplitGrid(Grid, D, Split, Left);
      bbox BBox{
        .Min = Params.BBox.Min + G.From3 * W3,
        .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
      };
      /* Write the refinement bits (NOTE: keep either this or the next block, not both) */
#if !defined(ZFP)
      FOR(int, I, 0, 3) { // write refinement bits
        while (G.Dims3[I] > 1 && BBox.Max[I] - BBox.Min[I] > Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
        //while (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
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
      }
#else
      vec3i Dims3((i32)G.Dims3.x, (i32)G.Dims3.y, (i32)G.Dims3.z);
      vec3i Stride3((i32)G.Stride3.x, (i32)G.Stride3.y, (i32)G.Stride3.z);
      vec3i From3((i32)G.From3.x, (i32)G.From3.y, (i32)G.From3.z);
      From3 = From3 + Dims3 * Stride3 / 2;
      vec3i B3 = Params.Dims3;
      auto* PCell = &ParticleCells[ROW3(B3.x, B3.y, B3.z, From3.x, From3.y, From3.z)];
      assert(PCell->ParticleId == -1);
      PCell->ParticleId = Begin;
      PCell->Grid = G;
      FOR(i32, I, 0, 3) {
        if (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
          PCell->DimsEncode |= (1 << I);
        }
      }
#endif
    } else { // recurse on the left
      bool RSplit = Split == ResolutionSplit;
      split_type NextSplit = (RSplit && Level > 1) ? ResolutionSplit : SpatialSplit;
      BuildTreeDFS(Begin, Mid, (Code * 2 + 1), SplitGrid(Grid, D, Split, Left), 
        Level - RSplit, NextSplit, Depth + 1);
    }
  }
  if (Mid < End) {
    if (Mid + 1 == End) { // right leaf
      vec3f P3 = Particles[Mid].Pos;
      auto G = SplitGrid(Grid, D, Split, Right);
      bbox BBox{
        .Min = Params.BBox.Min + G.From3 * W3,
        .Max = Params.BBox.Min + (G.From3 + G.Dims3) * W3
      };
      /* Write the refinement bits (NOTE: keep either this or the next block, not both) */
#if !defined(ZFP)
      FOR(int, I, 0, 3) {
        while (G.Dims3[I] > 1 && BBox.Max[I] - BBox.Min[I] > Params.Accuracy) { // NOTE: enable this to refine the tree uniformly until the leaf level
        //while (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
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
      }
#else
      vec3i From3((i32)G.From3.x, (i32)G.From3.y, (i32)G.From3.z);
      vec3i Dims3((i32)G.Dims3.x, (i32)G.Dims3.y, (i32)G.Dims3.z);
      vec3i Stride3((i32)G.Stride3.x, (i32)G.Stride3.y, (i32)G.Stride3.z);
      From3 = From3 + Dims3 * Stride3 / 2;
      vec3i B3 = Params.Dims3;
      auto* PCell = &ParticleCells[ROW3(B3.x, B3.y, B3.z, From3.x, From3.y, From3.z)];
      assert(PCell->ParticleId == -1);
      PCell->ParticleId = Mid;
      PCell->Grid = G;
      FOR(i32, I, 0, 3) {
        if (BBox.Max[I] - BBox.Min[I] > Params.Accuracy) {
          PCell->DimsEncode |= (1 << I);
        }
      }
#endif
    } else { // recurse on the right
      BuildTreeDFS(Mid, End, (Code * 2 + 2), SplitGrid(Grid, D, Split, Right), 
        Level, SpatialSplit, Depth + 1);
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
  else EXIT_ERROR(ErrorMsg);

  if (Params.Action == action::Encode) {
    if (!OptVal(Argc, Argv, "--name", &Params.OutFile)) EXIT_ERROR("missing --name");
    sprintf(Params.Name, "%s", Params.OutFile);
    if (!OptVal(Argc, Argv, "--ndims", &Params.NDims)) EXIT_ERROR("missin --ndims");
    if (!OptVal(Argc, Argv, "--nlevels", &Params.NLevels)) EXIT_ERROR("missing --nlevels");
    if (!OptVal(Argc, Argv, "--height", &Params.MaxHeight)) {
      if (!OptVal(Argc, Argv, "--accuracy", &Params.Accuracy))
        EXIT_ERROR("missing --height and --accuracy");
    }
    Params.NoRefinement = OptExists(Argc, Argv, "--no_refinement");
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
//    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    if (!OptVal(Argc, Argv, "--block", &Params.BlockBits)) EXIT_ERROR("missing --block");
    if (strstr(Params.InFile, ".xyz"))
      Particles = ReadXYZ(Params.InFile);
    else if (strstr(Params.InFile, ".dat"))
      Particles = ReadCosmo(Params.InFile);
    else if (strstr(Params.InFile, ".vtu"))
      Particles = ReadVtu(Params.InFile);
    else
      EXIT_ERROR("Unsupported file format");
    Params.NParticles = Particles.size();
    printf("number of particles = %zu\n", Particles.size());
    Params.BBox = ComputeBoundingBox(Particles);
    if (Params.BBox.Max.z == Params.BBox.Min.z)
      Params.BBox.Max.z = Params.BBox.Min.z + 1;
    Params.LogDims3 = ComputeGrid(&Particles, Params.BBox, 0, Particles.size(), 0, DimsStr);
    printf("%s\n", DimsStr);
    Params.BaseHeight = Params.LogDims3.x + Params.LogDims3.y + Params.LogDims3.z;
    Params.Dims3 = vec3i(1 << Params.LogDims3.x, 1 << Params.LogDims3.y, 1 << Params.LogDims3.z);
    grid Grid{.From3 = vec3f(0), .Dims3 = vec3f(Params.Dims3), .Stride3 = vec3f(1)};
    printf("bounding box = (" PRIvec3f ") - (" PRIvec3f ")\n", EXPvec3(Params.BBox.Min), EXPvec3(Params.BBox.Max));
    printf("log dims 3 = " PRIvec3i "\n", EXPvec3(Params.LogDims3));
    //Print(NResLevels - 1, 0, 0, 0, 0, Particles.size());
    BlockStreams.resize(Params.NLevels + 1);
    RefBlockStreams.resize(Params.MaxHeight - Params.BaseHeight);
    CurrBlocks.resize(Params.NLevels, 0);
    CurrRefBlocks.resize(Params.MaxHeight - Params.BaseHeight);
    FOR(u8, H, 0, CurrRefBlocks.size()) {
      CurrRefBlocks[H].Level = -1;
      CurrRefBlocks[H].BlockId = u64(-1);
    }
    BlockBytes.resize(Params.NLevels);
    //EncodeRoot(Particles.size()); // NOTE: old method
    //EncodeRootNew(Particles.size());
    /* compute the maximum height based on the accuracy */
    if (Params.MaxHeight == 255) {
      Params.MaxHeight = 0;
      vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
      while (W3.x > Params.Accuracy) { ++Params.MaxHeight; W3.x *= 0.5; }
      while (W3.y > Params.Accuracy) { ++Params.MaxHeight; W3.y *= 0.5; }
      while (W3.z > Params.Accuracy) { ++Params.MaxHeight; W3.z *= 0.5; }
    }
    Params.MaxHeight = MAX(Params.MaxHeight, Params.BaseHeight);
    Params.BlockDims3 = Params.Dims3;
    ParticleCells.resize(PROD(Params.BlockDims3));
    InitWrite(&BlockStream, 100000000); // 100 MB
    //Coder.InitWrite(100000000);
    WriteVarByte(&BlockStream, Particles.size());
    BuildTreeDFS(0, Particles.size(), 0, Grid, Params.NLevels - 1, 
      Params.NLevels > 1 ? ResolutionSplit : SpatialSplit, 0);
    CompressBlock();
    Flush(&BlockStream);
    i64 BlockStreamSize = Size(BlockStream);
    printf("done compression\n");
    /* immediately decode the stream (for testing) */
    InitRead(&BlockStream, BlockStream.Stream);
    i64 N = ReadVarByte(&BlockStream);
    printf("N = %lld\n", N);
    DecodeTreeDFS(0, N, 0, Grid, Params.NLevels - 1,
      Params.NLevels > 1 ? ResolutionSplit : SpatialSplit, 0);
    //DecompressBlock();
    //NaiveCompressUsingZfp(Params.Accuracy);
    //printf("compressed size = %lld bytes\n", Size(BlockStream) - (BitsSkipped / 8));
    printf("compressed size = %lld bytes\n", Size(BlockStream));
    RMSE = sqrt(RMSE / (N * Params.NDims));
    printf("RMSE = %f\n", RMSE);
    printf("Stream size = %lld\n", (StreamSize + 7) / 8 + BlockStreamSize);
    /* NOTE: old method
    BuildTreeInner(q_item{ .Begin = 0,
                           .End = (i64)Particles.size(),
                           .TreeIdx = 1,
                           .ResIdx = 0,
                           .NodeIdx = 1,
                           .ParIdx = 0,
                           .Grid = Grid,
                           .Level = i8(Params.NLevels - 1),
                           .Height = 0,
                           .SplitType = Params.NLevels > 1 ? ResolutionSplit : SpatialSplit }, Params.Accuracy);
    FlushBlocksToFiles();*/
    //BuildTreeNew(q_item_new { .Begin = 0,
                              //.End = (i64)Particles.size(),
                              //.Idx = 1,
                              //.Grid = Grid,
                              //.Height = 0 }, Params.Accuracy);
    //FlushBlocksToFilesNew();
    printf("nblocks written = %lld\n", NBlocksWritten);
  } else if (Params.Action == action::Decode) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    u8 MaxHeight = 0;
    f32 Accuracy = 0;
    if (!OptVal(Argc, Argv, "--height", &MaxHeight)) {
      if (!OptVal(Argc, Argv, "--accuracy", &Accuracy))
        EXIT_ERROR("missing --height and --accuracy");
    }
    OptVal(Argc, Argv, "--max_level", &Params.MaxLevel);
    OptVal(Argc, Argv, "--max_num_blocks", &Params.MaxNBlocks);
    OptVal(Argc, Argv, "--max_subsampling", &Params.MaxParticleSubSampling);

    ReadMetaFile(Params.InFile);
    if (Accuracy != 0) {
      MaxHeight = 0;
      vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
      while (W3.x > Accuracy) { ++MaxHeight; W3.x *= 0.5; }
      while (W3.y > Accuracy) { ++MaxHeight; W3.y *= 0.5; }
      while (W3.z > Accuracy) { ++MaxHeight; W3.z *= 0.5; }
    }
    Params.MaxHeight = MAX(MIN(Params.MaxHeight, MaxHeight), Params.BaseHeight);
    Params.Accuracy = Accuracy;
    BlockOffsets.resize(Params.NLevels);
    BlockBytes.resize(Params.NLevels + 1);
    BlockStreams.resize(Params.NLevels + 1);
    RefBlockStreams.resize(Params.MaxHeight - Params.BaseHeight);
    printf("baseheight = %d maxheight = %d\n", Params.BaseHeight, Params.MaxHeight);
    Blocks.resize(Params.NLevels + 1);
    Blocks[Params.NLevels].resize(1);
    Heap.insert(block_data{.Level = Params.NLevels, .Height = 0, .BlockId = 0}, block_priority{.Level = Params.NLevels, .BlockId = 0, .Error = 0});
    bool Continue = true;
    int NBlocks = 0;
    while (Continue && /*NBlocks < Params.MaxNBlocks*/BlockBytesRead < Params.MaxNBlocks) {
      //Continue = RefineByLevel();
      Continue = RefineByError();
      ++NBlocks;
    }
    if (!Blocks[Params.NLevels].empty()) {
      Particles.reserve(Blocks[Params.NLevels][0].Nodes[0]);
      i8 D = 0;
      grid Grid{.From3 = vec3f(0), .Dims3 = vec3f(Params.Dims3), .Stride3 = vec3f(1)};
      i8 Level = Params.NLevels - 1;
      u8 Height = 0;
      if (Params.NLevels == 1) {
        GenerateParticles(tree_node{
          .Level = Level,
          .Height = 0,
          .NodeId = 1,
          .Grid = Grid,
          .D = 0
        });
      } else {
        while (true) {
          if (Level <= Params.MaxLevel && 0 == GenerateParticles(tree_node{
            .Level = Level,
            .Height = u8(Height + 1),
            .NodeId = 1,
            .Grid = SplitGrid(Grid, D, ResolutionSplit, Right),
            .D = i8((D + 1) % Params.NDims)
          })) {
            //GenerateParticlesPerNode(Blocks[Params.NLevels][0].Nodes[LEVEL_TO_NODE(Level)], SplitGrid(Grid, D, ResolutionSplit, Right));
          }
          Grid = SplitGrid(Grid, D, ResolutionSplit, Left);
          D = (D + 1) % Params.NDims;
          --Level;
          ++Height;
          if (Level == 0) {
            GenerateParticles(tree_node{
              .Level = Level,
              .Height = Height,
              .NodeId = 1,
              .Grid = Grid,
              .D = D
            });
            break;
          } 
        }
      }
      printf("ncount = %lld %lld\n", NCount, NCount2);
      printf("bytes read = %lld\n", BlockBytesRead);
      WriteXYZ(Params.OutFile, Particles.begin(), Particles.end());
    }
  } else if (Params.Action == action::Error) {
    if (!OptVal(Argc, Argv, "--in", &Params.InFile)) EXIT_ERROR("missing --in");
    if (!OptVal(Argc, Argv, "--out", &Params.OutFile)) EXIT_ERROR("missing --out");
    if (!OptVal(Argc, Argv, "--dims", &Params.Dims3)) EXIT_ERROR("missing --dims");
    std::vector<particle> Particles1 = ReadXYZ(Params.InFile);
    std::vector<particle> Particles2 = ReadXYZ(Params.OutFile);
    f32 Err = Error(Particles1, Particles2, Params.Dims3);
    printf("error = %f\n", Err);
  }

  //RandomLevels(&Particles);
}

