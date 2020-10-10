#include "common.h"

static bitstream BlockStream; // compressed stream for the current block
static std::vector<block_meta> BlockBytesNew; // [block id] -> block size
static u64 CurrBlock = 0; // [level] -> current block id
static cdf_table CdfTable;
static arithmetic_coder<> Coder;
static int EncodedNodesCount = 0;

void
EncodeRootNew(i64 N) {
  InitWrite(&BlockStream, 1024);
  GrowToAccomodate(&BlockStream, 8);
  WriteVarByte(&BlockStream, N);
}

static void
WriteBlockNew(bitstream* Bs, u64 BlockIdx) {
  if (Size(*Bs) > 0) {
    Flush(Bs);
    FILE* Fp = fopen(PRINT("%s.bin", Params.OutFile), "ab");
    fwrite(Bs->Stream.Data, Size(*Bs), 1, Fp);
    fclose(Fp);

    // book-keeping
    BlockBytesNew.push_back(block_meta{.Size = Size(*Bs), .BlockId = BlockIdx});
    MaxBlockSize = MAX(MaxBlockSize, (int)Size(*Bs));
    Rewind(Bs);
    ++NBlocksWritten;
  }
}

void
FlushBlocksToFilesNew() {
  printf("block stream size = %lld\n", Size(BlockStream));
  printf("arithmetic stream size = %lld\n", Size(Coder.BitStream));
  printf("number of encoded nodes = %d\n", EncodedNodesCount);
  /* write the regular blocks */
  WriteBlockNew(&BlockStream, CurrBlock);
  if (Params.MaxHeight > Params.BaseHeight) { // flush refinement blocks
    printf("flushing refinement blocks\n");
    u64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(0); // TODO: check this
    FOR(u8, H, 0, Params.MaxHeight - Params.BaseHeight) {
      WriteBlockNew(&RefBlockStreams[H], CurrRefBlocks[H].BlockId + (H + 1) * NBlocksAtLeaf);
    }
  }
  // write an index consisting of all blocks in the file
  // TODO: compress the index?
  // TODO: if too many blocks have 0 bytes, maybe we can write a sparse index
  FILE* Fp = fopen(PRINT("%s.bin", Params.OutFile), "ab");
  Padding.resize(MaxBlockSize);
  fwrite(Padding.data(), Padding.size(), 1, Fp); // we write a padding the size of one block at the end of the file (why?)
  u64 NBlocks = BlockBytes.size();
  fwrite(BlockBytes.data(), sizeof(block_meta) * NBlocks, 1, Fp);
  fwrite(&NBlocks, sizeof(NBlocks), 1, Fp);
  fwrite(&MaxBlockSize, sizeof(MaxBlockSize), 1, Fp);
  printf("max block size = %d\n", MaxBlockSize);
  fclose(Fp);
  /* write the meta-data file */
  WriteMetaFile(Params, PRINT("%s.idx", Params.OutFile));
}


INLINE static void
EncodeNodeNew(i64 NodeIdx, i64 M, i64 N) {
  /* NOTE: uncomment to write to blocks
  u64 BlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  if (BlockIdx != CurrBlock) { // we have moved to the next block, dump the current block to disk
    WriteBlockNew(&BlockStream, CurrBlock);
    CurrBlock = BlockIdx;
  } */
  bitstream* Bs = &BlockStream;
  GrowToAccomodate(Bs, 8);
  EncodeCenteredMinimal((u32)N, (u32)M + 1, Bs);
  ++EncodedNodesCount;
  /* NOTE: comment out the following to disable the binomial coding */
  //f64 Mean = f64(M) / 2; // mean
  //f64 StdDev = sqrt(f64(M)) / 2; // standard deviation
  //EncodeRange(Mean, StdDev, f64(0), f64(M), f64(N), CdfTable, &BlockStream, &Coder);
}
    
/* Encode particle refinement bits */
static void
EncodeParticleNew(u64 NodeIdx, const vec3f& Pos, bbox BBox) {
  //vec3f P3 = Pos;
  //u8 H = Params.BaseHeight + 1;
  //i8 D = Params.BaseHeight % Params.NDims;
  //u64 BaseBlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  //while (H <= Params.MaxHeight) {
  //  i8 K = H - Params.BaseHeight - 1;
  //  if (CurrRefBlocks[K].BlockId == u64(-1))
  //    CurrRefBlocks[K].BlockId = BaseBlockIdx;
  //  if (CurrRefBlocks[K].Level == -1)
  //    CurrRefBlocks[K].Level = Level;
  //  //u64 BlockIdx = BaseBlockIdx + (K + 1) * NBlocksAtLeaf;
  //  bool NewBlock = BaseBlockIdx != CurrRefBlocks[K].BlockId;
  //  bool NewLevel = CurrRefBlocks[K].Level != Level;
  //  if (NewBlock || NewLevel) {
  //    i64 NBlocksAtLeaf = NUM_BLOCKS_AT_LEAF(CurrRefBlocks[K].Level);
  //    WriteBlock(&RefBlockStreams[K], CurrRefBlocks[K].Level, CurrRefBlocks[K].BlockId + (K + 1) * NBlocksAtLeaf);
  //    CurrRefBlocks[K].Level = Level;
  //    CurrRefBlocks[K].BlockId = BaseBlockIdx;
  //  }
  //  bitstream* Bs = &RefBlockStreams[K];
  //  GrowToAccomodate(Bs, 1);
  //  float Half = (BBox.Max[D] + BBox.Min[D]) * 0.5f;
  //  bool Left = P3[D] < Half;
  //  Write(Bs, Left);
  //  if (Left) BBox.Max[D] = Half;
  //  else BBox.Min[D] = Half;
  //  D = (D + 1) % Params.NDims;
  //  ++H;
  //}
}
/* Here we always use the "resolution" splits */
void
BuildTreeNew(q_item_new Q, float Accuracy) {
  /* NOTE: comment to disable the binomial coding */
  CdfTable = CreateBinomialTable(cutoff1);
  GrowToAccomodate(&BlockStream, 100000000); // 100 MB
  Coder.InitWrite(100000000);

  std::deque<q_item_new> Queue;
  Queue.push_back(Q);
  vec3f W3 = (Params.BBox.Max - Params.BBox.Min) / vec3f(Params.Dims3);
  while (!Queue.empty()) {
    Q = Queue.front();
    Queue.pop_front();
    REQUIRE(Q.Height <= Params.MaxHeight);
    i64 N = Q.End - Q.Begin;
    assert((N == 1) || IS_EVEN(int(Q.Grid.Dims3[Q.D])));
    i64 Mid = Q.Begin;
    vec3f Error3 = (W3 * Q.Grid.Dims3) / f64(N);
    bool Stop = Error3.x <= Accuracy && Error3.y <= Accuracy && (Params.NDims > 2 ? Error3.z <= Accuracy : true);
    if (Stop) continue;
    //if (N <= 1) continue; // enable this to stop the tree construction after the base height
    if (Q.Height < Params.BaseHeight && Q.End - Q.Begin > 1) { // NOTE: here we enqueue only nodes that have more than 1 particles
      auto Pred = [W3, &Q](const particle& P) {
        int Bin = MIN(Params.Dims3[Q.D] - 1, int((P.Pos[Q.D] - Params.BBox.Min[Q.D]) / W3[Q.D]));
        assert(IS_INT(Q.Grid.From3[Q.D]) && IS_INT(Q.Grid.Stride3[Q.D]) && IS_INT(Q.Grid.Dims3[Q.D]));
        REQUIRE((Bin - int(Q.Grid.From3[Q.D])) % int(Q.Grid.Stride3[Q.D]) == 0);
        Bin = (Bin - int(Q.Grid.From3[Q.D])) / int(Q.Grid.Stride3[Q.D]);
        return IS_EVEN(Bin);
      };
      Mid = partition(RANGE(Particles, Q.Begin, Q.End), Pred) - Particles.begin();
      //float S = (Q.Grid.Dims3[Q.D] > 1.5f) * (Q.Grid.Stride3[Q.D] - 1) + 1;
      //float M = Params.BBox.Min[Q.D] + W3[Q.D] * (Q.Grid.From3[Q.D] + Q.Grid.Dims3[Q.D] * 0.5f * S);
      //auto Pred = [M, &Q](const particle& P) { return P.Pos[Q.D] < M; };
      //Mid = partition(RANGE(Particles, Q.Begin, Q.End), Pred) - Particles.begin();
      /* encoding the children (left child in particular) */
      EncodeNodeNew(Q.Idx, Q.End - Q.Begin, Mid - Q.Begin);
      /* enqueue children */
      if (Q.Begin < Mid) { // left child
        Queue.push_back(q_item_new {
          .Begin = Q.Begin,
          .End = Mid,
          .Idx = Q.Idx * 2,
          //.Grid = SplitGrid(Q.Grid, Q.D, SpatialSplit, Left),
          .Grid = SplitGrid(Q.Grid, Q.D, ResolutionSplit, Left),
          .D = i8((Q.D + 1) % Params.NDims),
          .Height = u8(Q.Height + 1),
        });
      }
      if (Mid < Q.End) { // right child
        Queue.push_back(q_item_new {
          .Begin = Mid,
          .End = Q.End,
          .Idx = Q.Idx * 2 + 1,
          //.Grid = SplitGrid(Q.Grid, Q.D, SpatialSplit, Right),
          .Grid = SplitGrid(Q.Grid, Q.D, ResolutionSplit, Right),
          .D = i8((Q.D + 1) % Params.NDims),
          .Height = u8(Q.Height + 1),
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
      EncodeParticleNew(Q.Idx, Particles[Q.Begin].Pos, BBox);
    }
  }
  Coder.EncodeFinalize();
}
