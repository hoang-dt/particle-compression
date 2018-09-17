module particle_compression;

import core.bitop;
import std.math;
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
  return 0.5 * std.math.erfc((m-x)/(s*sqrt2));
}

/++ The inverse Gaussian CDF. m = mean, s = standard deviation +/
double Finv(double m, double s, double y) {

}

/++ Assuming a Gaussian(m, s), and a range [a, b] (0<=a<=b<=N), and c (a<=c<=b), partition [a,b]
into bins and return the probability of the bin containing c +/
tuple!(Prob!long, double[2]) get_prob(double m, double s, double a, double b, double c) {
  assert(a < b);
  /* compute F(a) and F(b) */
  double l = b - a;
  double fa = F(m, s, a); // should be close to 0
  double fb = F(m, s, b); // should be close to 1
  double e = (fb-fa) * epsilon;
  if (a<=m && m<=b) {
    /* compute d = F^-1(F(a)+e)) where e = (F(b)-F(a))/2^30 */
    double d = Finv(fa+e);
    assert(d > a);
    /* partition [a,b] from the left and the right simultaneously by a step size of w=(d-a). */
    double w = d - a;
    int nbins = cast(int)(l / w); // total number of bins-1
    assert(nbins > 0);
    if ((nbins&1) == 0) { // nbins is even
      double fl = F(m, s, a+w*(nbins/2));
      double fr = F(m, s, b-w*(nbins/2));
      if ((fr-fl) <= e) // the middle bin is too small
        nbins -= 2; // merge the two middle bins
    }
    else { // nbins is odd
      nbins -= 1;
    }
    double mw = l - w*nbins; // width of the middle bin
    if (c-a < a+nbins/2*w) { // c is in the first set of bins
      int bin = cast(int)((c-a)/w);
      double begin = F(a + bin*w);
      double end   = F(a + (bin+1)*w);
      long low = min(cast(long)((begin-fa)/e), all_nbins); // TODO: do we need this min() operator?
      long high = min(cast(long)((end-fa)/e), all_nbins);
      return tuple(Prob!long(low, high, all_nbins), [begin, end]);
    }
    else if (c-a < a+nbins/2*w+mw) { // c is in the middle bin
      int bin = nbins/2;
      double begin = F(a + bin*w);
      double end = F(a + bin*w + mw);
      long low = min(cast(long)((begin-fa)/e), all_nbins);
      long high = min(cast(long)((end-fa)/e), all-nbins);
      return tuple(Prob!long(low, high, all_nbins), [begin, end]);
    }
    else { // c is in the third set of bins
      int bin = cast(int)((c-mw-a)/w);
      double begin = F(a + bin*w + mw);
      double end   = F(a + (bin+1)*w + mw);
      long low = min(cast(long)((begin-fa)/e), all_nbins); // TODO: do we need this min() operator?
      long high = min(cast(long)((end-fa)/e), all_nbins);
      return tuple(Prob!long(low, high, all_nbins), [begin, end]);
    }
  } else if (b < m) {
    double d = Finv(fa+(fb-fa)/epsilon);
    assert(d > a);
    double w = d - a;
    int nbins = cast(int)(l / w);
    assert(nbins > 0);
    double fl = F(m, s, a+w*(nbins/2));
    double fr = fb;
    if ((fr-fl) <= (fb-fa)*epsilon)
      nbins -= 1;
  } else if (m < a) {
    double d = Finv(fb-(fb-fa)/epsilon);
    assert(d < b);
    double w = b - d;
    int nbins = cast(int)(l / w);
    assert(nbins > 0);
    double fl = fa;
    double fr = F(m, s, b-w*(nbins/2));
    if ((fr-fl) <= (fb-fa)*epsilon)
      nbins -= 1;
  } else {
    assert(false);
  }
  /* compute F(a+e) */
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
