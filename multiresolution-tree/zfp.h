/* Adapted from the zfp compression library */

#pragma once

#include "common.h"
#include <immintrin.h>
//#include <iostream>

#if defined(__clang__) || defined(__GNUC__)
#define idx2_Restrict __restrict__
#elif defined(_MSC_VER)
#define idx2_Restrict __restrict
#endif

extern const vec3i ZDims; /* 4 x 4 x 4 */

/* Forward/inverse zfp lifting in 1D */
template <typename t> void FLift(t* P, int S);
template <typename t> void ILift(t* P, int S);

  /* zfp transform in 3D. The input is assumed to be in row-major order. */
template <typename t> void ForwardZfp(t* P, int D);
template <typename t> void InverseZfp(t* P, int D);

/* Reorder coefficients within a zfp block, and convert them from/to negabinary */
template <typename t, typename u> void ForwardShuffle(t* IBlock, u* UBlock, int D);
template <typename t, typename u> void InverseShuffle(u* UBlock, t* IBlock, int D);

/* Pad partial block of width N < 4 and stride S */
template <typename t> void PadBlock1D(t* P, int N, int S);
template <typename t> void PadBlock2D(t* P, const vec2i& N);
template <typename t> void PadBlock3D(t* P, const vec3i& N);

template <typename t> void Encode(t* Block, int NVals, int B, i8& N, bitstream* Bs);
template <typename t> void Decode(t* Block, int NVals, int B, i8& N, bitstream* Bs);

struct bitstream;

/*
zfp lifting transform for 4 samples in 1D.
 non-orthogonal transform
        ( 4  4  4  4) (X)
 1/16 * ( 5  1 -1 -5) (Y)
        (-4  4  4 -4) (Z)
        (-2  6 -6  2) (W) */
        // TODO: look into range expansion for this transform
template <typename t> void
FLift(t* P, int S) {
  assert(P);
  assert(S > 0);
  t X = P[0 * S], Y = P[1 * S], Z = P[2 * S], W = P[3 * S];
  X += W; X >>= 1; W -= X;
  Z += Y; Z >>= 1; Y -= Z;
  X += Z; X >>= 1; Z -= X;
  W += Y; W >>= 1; Y -= W;
  W += Y >> 1; Y -= W >> 1;
  P[0 * S] = X; P[1 * S] = Y; P[2 * S] = Z; P[3 * S] = W;
}

/*
zfp inverse lifting transform for 4 samples in 1D.
NOTE: this lifting is not perfectly reversible
 non-orthogonal transform
       ( 4  6 -4 -1) (x)
 1/4 * ( 4  2  4  5) (y)
       ( 4 -2  4 -5) (z)
       ( 4 -6 -4  1) (w) */
template <typename t> void
ILift(t* P, int S) {
  assert(P);
  assert(S > 0);
  t X = P[0 * S], Y = P[1 * S], Z = P[2 * S], W = P[3 * S];
  Y += W >> 1; W -= Y >> 1;
  Y += W; W <<= 1; W -= Y;
  Z += X; X <<= 1; X -= Z;
  Y += Z; Z <<= 1; Z -= Y;
  W += X; X <<= 1; X -= W;
  P[0 * S] = X; P[1 * S] = Y; P[2 * S] = Z; P[3 * S] = W;
}

template <typename t> void
ForwardZfp(t* P, int D) {
  assert(P);
  switch (D) {
  case 3:
    /* transform along X */
    for (int Z = 0; Z < 4; ++Z)
      for (int Y = 0; Y < 4; ++Y)
        FLift(P + 4 * Y + 16 * Z, 1);
    /* transform along Y */
    for (int X = 0; X < 4; ++X)
      for (int Z = 0; Z < 4; ++Z)
        FLift(P + 16 * Z + 1 * X, 4);
    /* transform along Z */
    for (int Y = 0; Y < 4; ++Y)
      for (int X = 0; X < 4; ++X)
        FLift(P + 1 * X + 4 * Y, 16);
    break;
  case 2:
    /* transform along X */
    for (int Y = 0; Y < 4; ++Y)
      FLift(P + 4 * Y, 1);
    /* transform along Y */
    for (int X = 0; X < 4; ++X)
      FLift(P + 1 * X, 4);
    break;
  case 1:
    FLift(P, 1);
    break;
  default:
    break;
  };
}

template <typename t> void
InverseZfp(t* P, int D) {
  assert(P);
  switch (D) {
  case 3:
    /* transform along Z */
    for (int Y = 0; Y < 4; ++Y)
      for (int X = 0; X < 4; ++X)
        ILift(P + 1 * X + 4 * Y, 16);
    /* transform along y */
    for (int X = 0; X < 4; ++X)
      for (int Z = 0; Z < 4; ++Z)
        ILift(P + 16 * Z + 1 * X, 4);
    /* transform along X */
    for (int Z = 0; Z < 4; ++Z)
      for (int Y = 0; Y < 4; ++Y)
        ILift(P + 4 * Y + 16 * Z, 1);
    break;
  case 2:
    /* transform along y */
    for (int X = 0; X < 4; ++X)
      ILift(P + 1 * X, 4);
    /* transform along X */
    for (int Y = 0; Y < 4; ++Y)
      ILift(P + 4 * Y, 1);
    break;
  case 1:
    ILift(P, 1);
    break;
  default:
    break;
  };
}

template <int S>
struct perm2 {
  inline static const std::array<int, S* S> Table = []() {
    std::array<int, S* S> Arr;
    int I = 0;
    for (int Y = 0; Y < S; ++Y) {
      for (int X = 0; X < S; ++X) {
        Arr[I++] = Y * S + X;
      }
    }
    for (I = 0; I < Arr.size(); ++I) {
      for (int J = I + 1; J < Arr.size(); ++J) {
        int XI = Arr[I] % S, YI = Arr[I] / S;
        int XJ = Arr[J] % S, YJ = Arr[J] / S;
        if (XI + YI > XJ + YJ) {
          std::swap(Arr[I], Arr[J]);
        } else if ((XI + YI == XJ + YJ) && (XI * XI + YI * YI > XJ * XJ + YJ * YJ)) {
          std::swap(Arr[I], Arr[J]);
        }
      }
    }
    return Arr;
  }();
};

/*
Use the following array to reorder transformed coefficients in a zfp block.
The ordering is first by i + j + k, then by i^2 + j^2 + k^2. */
#define idx2_Index(i, j, k) ((i) + 4 * (j) + 16 * (k))
constexpr i8
  Perm3[64] = {
    idx2_Index(0, 0, 0), /*  0 : 0 */

    idx2_Index(1, 0, 0), /*  1 : 1 */
    idx2_Index(0, 1, 0), /*  2 : 1 */
    idx2_Index(0, 0, 1), /*  3 : 1 */

    idx2_Index(0, 1, 1), /*  4 : 2 */
    idx2_Index(1, 0, 1), /*  5 : 2 */
    idx2_Index(1, 1, 0), /*  6 : 2 */
    idx2_Index(2, 0, 0), /*  7 : 2 */
    idx2_Index(0, 2, 0), /*  8 : 2 */
    idx2_Index(0, 0, 2), /*  9 : 2 */

    idx2_Index(1, 1, 1), /* 10 : 3 */
    idx2_Index(2, 1, 0), /* 11 : 3 */
    idx2_Index(2, 0, 1), /* 12 : 3 */
    idx2_Index(0, 2, 1), /* 13 : 3 */
    idx2_Index(1, 2, 0), /* 14 : 3 */
    idx2_Index(1, 0, 2), /* 15 : 3 */
    idx2_Index(0, 1, 2), /* 16 : 3 */
    idx2_Index(3, 0, 0), /* 17 : 3 */
    idx2_Index(0, 3, 0), /* 18 : 3 */
    idx2_Index(0, 0, 3), /* 19 : 3 */

    idx2_Index(2, 1, 1), /* 20 : 4 */
    idx2_Index(1, 2, 1), /* 21 : 4 */
    idx2_Index(1, 1, 2), /* 22 : 4 */
    idx2_Index(0, 2, 2), /* 23 : 4 */
    idx2_Index(2, 0, 2), /* 24 : 4 */
    idx2_Index(2, 2, 0), /* 25 : 4 */
    idx2_Index(3, 1, 0), /* 26 : 4 */
    idx2_Index(3, 0, 1), /* 27 : 4 */
    idx2_Index(0, 3, 1), /* 28 : 4 */
    idx2_Index(1, 3, 0), /* 29 : 4 */
    idx2_Index(1, 0, 3), /* 30 : 4 */
    idx2_Index(0, 1, 3), /* 31 : 4 */

    idx2_Index(1, 2, 2), /* 32 : 5 */
    idx2_Index(2, 1, 2), /* 33 : 5 */
    idx2_Index(2, 2, 1), /* 34 : 5 */
    idx2_Index(3, 1, 1), /* 35 : 5 */
    idx2_Index(1, 3, 1), /* 36 : 5 */
    idx2_Index(1, 1, 3), /* 37 : 5 */
    idx2_Index(3, 2, 0), /* 38 : 5 */
    idx2_Index(3, 0, 2), /* 39 : 5 */
    idx2_Index(0, 3, 2), /* 40 : 5 */
    idx2_Index(2, 3, 0), /* 41 : 5 */
    idx2_Index(2, 0, 3), /* 42 : 5 */
    idx2_Index(0, 2, 3), /* 43 : 5 */

    idx2_Index(2, 2, 2), /* 44 : 6 */
    idx2_Index(3, 2, 1), /* 45 : 6 */
    idx2_Index(3, 1, 2), /* 46 : 6 */
    idx2_Index(1, 3, 2), /* 47 : 6 */
    idx2_Index(2, 3, 1), /* 48 : 6 */
    idx2_Index(2, 1, 3), /* 49 : 6 */
    idx2_Index(1, 2, 3), /* 50 : 6 */
    idx2_Index(0, 3, 3), /* 51 : 6 */
    idx2_Index(3, 0, 3), /* 52 : 6 */
    idx2_Index(3, 3, 0), /* 53 : 6 */

    idx2_Index(3, 2, 2), /* 54 : 7 */
    idx2_Index(2, 3, 2), /* 55 : 7 */
    idx2_Index(2, 2, 3), /* 56 : 7 */
    idx2_Index(1, 3, 3), /* 57 : 7 */
    idx2_Index(3, 1, 3), /* 58 : 7 */
    idx2_Index(3, 3, 1), /* 59 : 7 */

    idx2_Index(2, 3, 3), /* 60 : 8 */
    idx2_Index(3, 2, 3), /* 61 : 8 */
    idx2_Index(3, 3, 2), /* 62 : 8 */

    idx2_Index(3, 3, 3), /* 63 : 9 */
};
#undef idx2_Index

#define idx2_Index(i, j) ((i) + 4 * (j))
constexpr i8
  Perm2[16] = {
    idx2_Index(0, 0), /*  0 : 0 */

    idx2_Index(1, 0), /*  1 : 1 */
    idx2_Index(0, 1), /*  2 : 1 */

    idx2_Index(1, 1), /*  3 : 2 */
    idx2_Index(2, 0), /*  4 : 2 */
    idx2_Index(0, 2), /*  5 : 2 */

    idx2_Index(2, 1), /*  6 : 3 */
    idx2_Index(1, 2), /*  7 : 3 */
    idx2_Index(3, 0), /*  8 : 3 */
    idx2_Index(0, 3), /*  9 : 3 */

    idx2_Index(2, 2), /* 10 : 4 */
    idx2_Index(3, 1), /* 11 : 4 */
    idx2_Index(1, 3), /* 12 : 4 */

    idx2_Index(3, 2), /* 13 : 5 */
    idx2_Index(2, 3), /* 14 : 5 */

    idx2_Index(3, 3), /* 15 : 6 */
  };
#undef idx2_Index


template <typename t, typename u> void
InverseShuffle(u* idx2_Restrict UBlock, t* idx2_Restrict IBlock, int D) {
  auto Mask = traits<u>::NBinaryMask;
  switch (D) {
  case 3: for (int I = 0; I < 64; ++I) IBlock[Perm3[I]] = (t)((UBlock[I] ^ Mask) - Mask); break;
  case 2: for (int I = 0; I < 16; ++I) IBlock[Perm2[I]] = (t)((UBlock[I] ^ Mask) - Mask); break;
  case 1: for (int I = 0; I < 4; ++I) IBlock[I] = (t)((UBlock[I] ^ Mask) - Mask); break;
  case 0: for (int I = 0; I < 1; ++I) IBlock[I] = (t)((UBlock[I] ^ Mask) - Mask); break;
  default: assert(false);
  };
}

template <typename t, typename u> void
ForwardShuffle(t* idx2_Restrict IBlock, u* idx2_Restrict UBlock, int D) {
  auto Mask = traits<u>::NBinaryMask;
  switch (D) {
  case 3: for (int I = 0; I < 64; ++I) UBlock[I] = (u)((IBlock[Perm3[I]] + Mask) ^ Mask); break;
  case 2: for (int I = 0; I < 16; ++I) UBlock[I] = (u)((IBlock[Perm2[I]] + Mask) ^ Mask); break;
  case 1: for (int I = 0; I < 4; ++I) UBlock[I] = (u)((IBlock[I] + Mask) ^ Mask); break;
  case 0: for (int I = 0; I < 1; ++I) UBlock[I] = (u)((IBlock[I] + Mask) ^ Mask); break;
  default: assert(false);
  };
}
  // TODO: this function is only correct for block size 4
template <typename t> void
PadBlock1D(t* P, int N, int S) {
  assert(P);
  assert(0 <= N && N <= 4);
  assert(S > 0);
  switch (N) {
  case 0:
    P[0 * S] = 0; /* fall through */
  case 1:
    P[1 * S] = P[0 * S]; /* fall through */
  case 2:
    P[2 * S] = P[1 * S]; /* fall through */
  case 3:
    P[3 * S] = P[0 * S]; /* fall through */
  default:
    break;
  }
}

template <typename t> void
PadBlock3D(t* P, const vec3i& N) {
  for (int Z = 0; Z < 4; ++Z)
    for (int Y = 0; Y < 4; ++Y)
      PadBlock1D(P + Z * 16 + Y * 4, N.x, 1);

  for (int Z = 0; Z < 4; ++Z)
    for (int X = 0; X < 4; ++X)
      PadBlock1D(P + Z * 16 + X * 1, N.y, 4);

  for (int Y = 0; Y < 4; ++Y)
    for (int X = 0; X < 4; ++X)
      PadBlock1D(P + Y * 4 + X * 1, N.z, 16);
}

template <typename t> void
PadBlock2D(t* P, const vec2i& N) {
  for (int Y = 0; Y < 4; ++Y)
    PadBlock1D(P + Y * 4, N.x, 1);
  for (int X = 0; X < 4; ++X)
    PadBlock1D(P + X * 1, N.y, 4);
}

// NOTE: this is the one being used
template <typename t> void
Encode(t* idx2_Restrict Block, int NVals, int B, /*i64 S, */i8& N, bitstream* idx2_Restrict BsIn) {
  //static_assert(is_unsigned<t>::Value);
  assert(NVals <= 64); // e.g. 4x4x4, 4x4, 8x8
  bitstream Bs = *BsIn;
  u64 X = 0;
  for (int I = 0; I < NVals; ++I)
    X += u64((Block[I] >> B) & 1u) << I;
  //  i8 P = (i8)Min((i64)N, S - BitSize(Bs));
  i8 P = N;
  if (P > 0) {
    WriteLong(&Bs, X, P);
    X >>= P; // P == 64 is fine since in that case we don't need X any more
  }
  for (; /*BitSize(Bs) < S &&*/ N < NVals;) {
    if (Write(&Bs, !!X)) { // group is significant
      for (; /*BitSize(Bs) < S &&*/ N + 1 < NVals;) {
        if (Write(&Bs, X & 1u)) { // found a significant coeff, break and retest
          break;
        } else { // have not found a significant coeff, continue until we find one
          X >>= 1;
          ++N;
        }
      }
      //      if (BitSize(Bs) >= S)
      //        break;
      X >>= 1;
      ++N;
    } else {
      break;
    }
  }
  *BsIn = Bs;
}

// NOTE: This is the one being used
template <typename t> void
Decode(t* idx2_Restrict Block, int NVals, int B, /*i64 S, */i8& N, bitstream* idx2_Restrict BsIn) {
    //static_assert(is_unsigned<t>::Value);
  assert(NVals <= 64); // e.g. 4x4x4, 4x4, 8x8
  bitstream Bs = *BsIn;
  //  i8 P = (i8)Min((i64)N, S - BitSize(Bs));
  i8 P = N;
  u64 X = P > 0 ? ReadLong(&Bs, P) : 0;
  for (; /*BitSize(Bs) < S &&*/ N < NVals;) {
    if (Read(&Bs)) {
      for (; /*BitSize(Bs) < S &&*/ N + 1 < NVals;)
        if (Read(&Bs)) break; else ++N;
      //if (BitSize(Bs) >= S)
      //  break;
      X += 1ull << (N++);
    }
    else {
      break;
    }
  }
  for (int I = 0; X; ++I, X >>= 1)
    Block[I] += (t)(X & 1u) << B;
  *BsIn = Bs;
}

