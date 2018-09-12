module particle_compression;

import core.bitop;
import std.math;
import bit_stream;
import kdtree;
import math;
import stats;

/++ v is from 0 to n-1 +/
void encode_centered_minimal(int v, int n, ref BitStreamNaive bs) {
  assert(n > 0);
  assert(v<n && v>=0);
  import std.stdio;
  int l1 = bsr(n);
  int l2 = ((1<<l1)==n) ? l1 : l1+1;
  int d = (1<<l2) - n;
  int m = (n-d) / 2;
  // TODO: reverse the bits before writing to the stream
  if (v < m)
    bs.write(v, l2);
    //writefln("%04b", v);
  else if (v >= m+d)
    bs.write(v-d, l2);
    //writefln("%04b", v-d);
  else  // middle
    bs.write(v, l1);
    //writefln("%03b", v);
}

int decode_centered_minimal(int n, ref BitStream bs) {
  assert(n > 0);
  import std.stdio;
  int l1 = bsr(n);
  int l2 = ((1<<l1)==n) ? l1 : l1+1;
  int d = (1<<l2) - n;
  int m = (n-d) / 2;
  bs.refill(); // TODO: minimize the number of refill
  int v = cast(int)bs.peek(l2);
  if (v < m) {
    bs.consume(l2);
    return v;
  }
  else if (v < 2*m) {
    bs.consume(l2);
    return v+d;
  }
  else {
    bs.consume(l1);
    return v >> 1;
  }
}

int group_bins(int N)(float[N] input, ref float[N] output) {
  int j = 1;
  output[0] = input[0];
  for (int i = 1; i+1 < input.length; ++i) {
    if (cast(int)input[i] != cast(int)input[i-1]) {
      output[j] = input[i];
      ++j;
    }
  }
  return j+1;
}

void encode(T)(Vec3!T[] positions, ref BitStreamNaive bs) {
  void traverse_encode(T,int R)(KdTree!(T,R) parent, ref BitStreamNaive bs) {
    int N = parent.end_ - parent.begin_;
    if (N == 1) { // leaf level
      // do nothing
    }
    else { // non leaf
      int n = parent.left_ is null ? 0 : parent.left_.end_ - parent.left_.begin_;
      // TODO: loop through the precomputed table and find the bin for n
      float m = float(N) / 2; // mean
      float s = sqrt(float(N)) / 2; // standard deviation
      for (int i = 0; i+1 < nbins; ++i) {
        lookup_table[i] = lookup_table[i] * s + m;
        if (lookup_table[i] > N)
          lookup_table[i] = N;
      }
      int actual_nbins = group_bins(lookup_table, small_lookup_table);
      int bin = 0;
      for (; bin+1 < nbins; ++bin) {
        if (lookup_table[bin] > n)
          break;
      }
      //if (N > nbins)
      //bs.write(bin, cast(int)ceil(log2(actual_nbins))); // encode the bits for the bin
      encode_centered_minimal(bin, nbins, bs);
      //bs.write(bin, cast(int)log2());
      //else
        //encode_centered_minimal(n, N+1, bs);
      int first = bin > 0 ? cast(int)ceil(lookup_table[bin-1]) : 0; // TODO: check the logic here (what if ceil(x) == x)
      int last = bin+1 < nbins ? cast(int)lookup_table[bin] : N;
      assert(first <= N);
      if (last > first)
       // bs.write(n-first, cast(int)log2(last-first+1));
        encode_centered_minimal(n-first, last-first+1, bs); // encode the difference from the bin's first value
      //encode_centered_minimal(n,N+1,bs);
      //bs.write(n, cast(int)ceil(log2(N+1)));
        assert(n-first <= n);
        assert(last-first<=N);
        //bs.write(n-first,cast(int)log2(last-first+1));
      if (parent.left_)
        traverse_encode(parent.left_, bs);
      if (parent.right_)
        traverse_encode(parent.right_, bs);
    }
  }
  auto tree = new KdTree!T();
  tree.build!"xyz"(positions);

  // TODO: first, encode the total number of particles
  /* pre-compute and store a table of inverse gaussian(0,1) cdf */
  const int nbits = 10;
  const int nbins = 1<<nbits;
  float[nbins] lookup_table;
  float[nbins] small_lookup_table;
  float step = 1.0f / nbins;
  for (int i = 0; i < nbins; ++i) {
    float y = (i+1) * step;
    lookup_table[i] = erfinv(2*y-1) * sqrt(2.0f);
  }
  bs.init_write(100000000); // 100 MB
  traverse_encode(tree, bs);
  bs.flush();
}
