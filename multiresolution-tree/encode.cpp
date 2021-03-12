#include "common.h"

static std::vector<block_meta> BlockBytesNew; // [block id] -> block size
static u64 CurrBlock = 0; // [level] -> current block id
static cdf_table CdfTable;
static arithmetic_coder<> Coder;
static int EncodedNodesCount = 0;

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

