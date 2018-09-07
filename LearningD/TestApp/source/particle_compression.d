module particle_compression;

import core.bitop;
import std.math;
import bit_stream;
import kdtree;
import math;
import stats;

/++ v is from 0 to n-1 +/
void encode_centered_minimal(int v, int n, ref BitStream bs) {
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

void encode(T)(Vec3!T[] positions, ref BitStream bs) {
  import std.stdio;
  void traverse_encode(T,int R)(KdTree!(T,R) parent, ref BitStream bs) {
    int N = parent.end_ - parent.begin_;
    if (N == 1) { // leaf level
      // do nothing
    }
    else { // non leaf
      int n = parent.left_ is null ? 0 : parent.left_.end_ - parent.left_.begin_;
      float m = float(N) / 2; // mean
      float s = sqrt(float(N)) / 2; // standard deviation
      float bin_width = 0.5f * (erfc(-0.5f/s/sqrt(2.0f)) - erfc(0.5f/s/sqrt(2.0f)));
      int nbins = cast(int)ceil(1.0f / bin_width);
      //writeln(nbins);
      if (nbins > lookup_table.length)
        lookup_table.length = nbins;
      int bin = 0;
      for (; bin+1 < nbins; ++bin) {
        float y = (bin+1) * bin_width;
        lookup_table[bin] = (erfinv(2*y-1) * sqrt(2.0f)) * s + m;
        writeln(lookup_table[bin]);
        if (lookup_table[bin] > n)
          break;
      }
      if (N > nbins)
        encode_centered_minimal(bin, nbins, bs);
      else
        encode_centered_minimal(n, N+1, bs);
      int first = bin > 0 ? cast(int)(ceil(lookup_table[bin-1])) : 0;
      int last = bin+1 < nbins ? cast(int)(lookup_table[bin]) : N;
      if (last > first)
        encode_centered_minimal(n-first, last-first+1, bs); // encode the difference from the bin's first value
      //encode_centered_minimal(n,N+1,bs);
      if (parent.left_)
        traverse_encode(parent.left_, bs);
      if (parent.right_)
        traverse_encode(parent.right_, bs);
    }
  }
  float[] lookup_table = new float[](1024);
  auto tree = new KdTree!T();
  tree.build!"xyz"(positions);

  // TODO: first, encode the total number of particles
  /* pre-compute and store a table of inverse gaussian(0,1) cdf */
  bs.init_write(100000000); // 100 MB
  traverse_encode(tree, bs);
  bs.flush();
}
