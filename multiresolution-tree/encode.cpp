#include "common.h"

INLINE static void /* encode a resolution node */
EncodeResNodeNew(i64 M, i64 N) {
//  // TODO: use binomial coding
//  GrowToAccomodate(&BlockStreams[Params.NLevels], 8);
////  WriteVarByte(&BlockStreams[Params.NLevels], N);
//  EncodeCenteredMinimal((u32)N, (u32)M + 1, &BlockStreams[Params.NLevels]);
}

//bitstream* Bs = Height <= Params.BaseHeight ? &BlockStreams[Level] : &RefBlockStreams[Height - Params.BaseHeight - 1].Bs;
INLINE static void
EncodeNodeNew(i8 Level, i64 NodeIdx, i64 M, i64 N) {
//  u64 BlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
////  printf("+++++++ encoding level = %d block = %llu\n", Level, BlockIdx);
//  if (BlockIdx != CurrBlocks[Level]) { // we have moved to the next block, dump the current block to disk
//    WriteBlock(&BlockStreams[Level], Level, CurrBlocks[Level]);
//    CurrBlocks[Level] = BlockIdx;
//  }
//  bitstream* Bs = &BlockStreams[Level];
//  GrowToAccomodate(Bs, 8);
////  WriteVarByte(Bs, N);
//  EncodeCenteredMinimal((u32)N, (u32)M + 1, Bs);
}

/* Encode particle refinement bits */
static void
EncodeParticleNew(i8 Level, u64 NodeIdx, const vec3f& Pos, bbox BBox) {
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


static void
BuildTreeNew(const params& Params, std::vector<particle>& Particles, q_item Q, float Accuracy) {
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
      auto Pred = [Params, W3, &Q](const particle& P) {
        int Bin = MIN(Params.Dims3[Q.D] - 1, int((P.Pos[Q.D] - Params.BBox.Min[Q.D]) / W3[Q.D]));
        assert(IS_INT(Q.Grid.From3[Q.D]) && IS_INT(Q.Grid.Stride3[Q.D]) && IS_INT(Q.Grid.Dims3[Q.D]));
        REQUIRE((Bin - int(Q.Grid.From3[Q.D])) % int(Q.Grid.Stride3[Q.D]) == 0);
        Bin = (Bin - int(Q.Grid.From3[Q.D])) / int(Q.Grid.Stride3[Q.D]);
        return IS_EVEN(Bin);
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
        EncodeResNodeNew(Q.End - Q.Begin, Mid - Q.Begin);
      } else {
        EncodeNodeNew(Q.Level - (Q.SplitType == ResolutionSplit), Q.SplitType == ResolutionSplit ? Q.NodeIdx : Q.NodeIdx * 2, Q.End - Q.Begin, Mid - Q.Begin);
      }
      /* enqueue children */
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
    } else { // Q.Height == Params.BaseHeight
      /* encoding the refinement bits */
      REQUIRE(N == 1);
      assert(Q.Grid.Dims3.x <= 1 && Q.Grid.Dims3.y <= 1 && Q.Grid.Dims3.z <= 1);
      // TODO: sometimes there are numerical issues where the particle is outside of BBox
      bbox BBox{
        .Min = Params.BBox.Min + Q.Grid.From3 * W3,
        .Max = Params.BBox.Min + (Q.Grid.From3 + Q.Grid.Dims3) * W3
      };
      EncodeParticleNew(Q.Level, Q.NodeIdx, Particles[Q.Begin].Pos, BBox);
    }
  }
}
