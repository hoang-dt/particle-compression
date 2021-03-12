#include "common.h"

static u64 CurrBlock = 0; // [level] -> current block id
static cdf_table CdfTable;
static arithmetic_coder<> Coder;
static int EncodedNodesCount = 0;

INLINE static void
EncodeNodeNew(i64 NodeIdx, i64 M, i64 N) {
  /* NOTE: uncomment to write to blocks
  u64 BlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  if (BlockIdx != CurrBlock) { // we have moved to the next block, dump the current block to disk
    WriteBlockNew(&BlockStream, CurrBlock);
    CurrBlock = BlockIdx;
  } */
  bitstream* Bs = &BlockStream;
  //GrowToAccomodate(Bs, 8);
  //EncodeCenteredMinimal((u32)N, (u32)M + 1, Bs);
  //++EncodedNodesCount;
  /* NOTE: comment out the following to disable the binomial coding */
  f64 Mean = f64(M) / 2; // mean
  f64 StdDev = sqrt(f64(M)) / 2; // standard deviation
  EncodeRange(Mean, StdDev, f64(0), f64(M), f64(N), CdfTable, &BlockStream, &Coder);
}
    
/* Encode particle refinement bits */
// TODO: need to refer to the global D array
static void
EncodeParticleNew(u64 NodeIdx, const vec3f& Pos, bbox BBox) {
  ++NParticlesDecoded;
  vec3f P3 = Pos;
  u8 H = Params.BaseHeight + 1;
  i8 D = Params.BaseHeight % Params.NDims; // TODO: use the correct one
  u64 BaseBlockIdx = NODE_TO_BLOCK_INDEX(NodeIdx);
  //while (/*H <= Params.MaxHeight*/BBox.Max[D] - BBox.Min[D] >= 2 * Params.Accuracy) {
  while (ceil(BBox.Min[D]) != floor(BBox.Max[D])) { // lossless
    i8 K = H - Params.BaseHeight - 1;
    //bitstream* Bs = &RefBlockStreams[K];
    bitstream* Bs = &BlockStream;
    GrowToAccomodate(Bs, 1);
    float Half = (BBox.Max[D] + BBox.Min[D]) * 0.5f;
    bool Left = P3[D] < Half;
    Write(Bs, Left);
    if (Left) BBox.Max[D] = Half;
    else BBox.Min[D] = Half;
    D = (D + 1) % Params.NDims; // TODO use the correct one
    ++H;
  }
  //REQUIRE(BBox.Max[D] - BBox.Min[D] < Params.Accuracy);
}
/* Here we always use the "resolution" splits */
// TODO: need to refer to the global D array

