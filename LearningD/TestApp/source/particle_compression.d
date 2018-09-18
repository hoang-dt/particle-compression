module particle_compression;

import core.bitop;
import std.mathspecial;
import std.typecons;
import arithmetic_coder;
import bit_stream;
import kdtree;
import math;
import stats;

const double sqrt2 = sqrt(2.0);
const long all_nbins = 1 << 30;
const double epsilon = 1.0 / cast(double)(all_nbins);

/++ The Gaussian CDF. m = mean, s = standard deviation +/
double F(double m, double s, double x) {
  return 0.5 * std.mathspecial.erfc((m-x)/(s*sqrt2));
}

/++ The inverse Gaussian CDF. m = mean, s = standard deviation +/
double Finv(double m, double s, double y) {
  return m + s * (erfinv(2*y-1)*sqrt2);
}

/++ Assuming a Gaussian(m, s), and a range [a, b] (0<=a<=b<=N), and c (a<=c<=b), partition [a,b]
into two bins of equal probability +/
void encode(double m, double s, double a, double b, double c, ref BitStream bs) {
  assert(a < b);
  /* compute F(a) and F(b) */
  double fa = F(m, s, a);
  double fb = F(m, s, b);
  /* compute F^-1((fa+fb)/2) */
  double mid = Finv(m, s, (fa+fb)*0.5);
  if (a==mid || b==mid) {
    int beg = cast(int)ceil(a);
    int end = cast(int)floor(b);
    int v = cast(int)c-beg;
    int n = end - beg + 1;
    return encode_centered_minimal(v, n, bs);
  }
  assert(a<=mid && mid<=b);
  if (c < mid) {
    bs.write(0);
    if (a+1 < mid)
      return encode(m, s, a, mid, c, bs);
  }
  else {
    bs.write(1);
    if (mid+1 < b)
      return encode(m, s, mid, b, c, bs);
  }
}

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
      encode(m, s, 0, N, n, bs);
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
