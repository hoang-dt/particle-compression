module particle_compression;

import core.bitop;
import std.mathspecial;
import std.typecons;
import arithmetic_coder;
import bit_stream;
import kdtree;
import math;
import stats;
import bit_ops;

const double sqrt2 = sqrt(2.0);
const long all_nbins = 1 << 30;
const double epsilon = 1.0 / cast(double)(all_nbins);

// TODO: count the number of bits per level

/++ The Gaussian CDF. m = mean, s = standard deviation +/
double F(double m, double s, double x) {
  return 0.5 * std.mathspecial.erfc((m-x)/(s*sqrt2));
}

/++ The inverse Gaussian CDF. m = mean, s = standard deviation +/
double Finv(double m, double s, double y) {
  return m + s * (erfinv(2*y-1)*sqrt2);
}

const int cutoff = 32;
alias Coder = ArithmeticCoder!();

void encode_binomial_small_range(int n, int v, in uint[] cdf_table, ref Coder coder) {
  assert(v>=0 && v<=n);
  uint lo = v==0 ? 0 : cdf_table[v-1];
  uint hi = cdf_table[v];
  uint scale = 1 << n;
  Prob!uint prob = Prob!uint(lo, hi, scale);
  coder.encode(prob);
}

int decode_binomial_small_range(int n, in uint[] cdf_table, ref Coder coder) {
  size_t v = coder.decode(cdf_table);
  assert(v<=n);
  return cast(int)v;
}

/++ Assuming a Gaussian(m, s), and a range [a, b] (0<=a<=b<=N), and c (a<=c<=b), partition [a,b]
into two bins of equal probability +/
void encode_range(double m, double s, double a, double b, double c, in uint[][] cdf_table, ref BitStream bs, ref Coder coder) {
  import std.stdio;
  assert(a <= b);
  bool first = true;
  int written = 0;
  while (true) {
    int beg = cast(int)ceil(a);
    int end = cast(int)floor(b);
    if (beg == end)
      return; // no need to write any bit
    if (end-beg+1 <= cutoff) {
      int v = cast(int)c-beg;
      int n = end - beg + 1; // v can be from 0 to n-1
      if (first) {
        assert(beg == 0);
        return encode_binomial_small_range(n-1, v, cdf_table[n-1], coder);
      }
      else
        return encode_centered_minimal(v, n, bs);
    }
    /* compute F(a) and F(b) */
    double fa = F(m, s, a);
    double fb = F(m, s, b);
    // TODO: what if fa==fb
    /* compute F^-1((fa+fb)/2) */
    double mid = Finv(m, s, (fa+fb)*0.5);
    if (mid<a || mid>b) // mid can be infinity when (fa+fb) == 0
      mid = a;
    if (a==mid || b==mid) {
      int v = cast(int)c-beg;
      int n = end - beg + 1;
      return encode_centered_minimal(v, n, bs);
    }
    assert(a<=mid && mid<=b);
    if (c < mid) {
      bs.write(0);
      if (written++ < 10)
        write(0);
      b = floor(mid);
    }
    else { // c >= mid
      bs.write(1);
      if (written++ < 10)
        write(1);
      a = ceil(mid);
    }
    first = false;
  }
}

/++ The inverse of encode +/
// TODO: refactor to put part the logic of this function to the decode function
int decode_range(double m, double s, double a, double b, uint[][] cdf_table, ref BitStream bs, ref Coder coder) {
  import std.stdio;
  assert(a <= b);
  bool first = true;
  while (true) {
    int beg = cast(int)ceil(a);
    int end = cast(int)floor(b);
    if (beg == end)
      return beg; // no need to write any bit
    if (end-beg+1 <= cutoff) {
      int n = end - beg + 1; // v can be from 0 to n-1
      if (first)
        return decode_binomial_small_range(n-1, cdf_table[n-1], coder);
      else
        return beg + decode_centered_minimal(n, bs);
    }
    /* compute F(a) and F(b) */
    double fa = F(m, s, a);
    double fb = F(m, s, b);
    // TODO: what if fa==fb
    /* compute F^-1((fa+fb)/2) */
    double mid = Finv(m, s, (fa+fb)*0.5);
    if (mid<a || mid>b) // mid can be infinity when (fa+fb) == 0
      mid = a;
    if (a==mid || b==mid) {
      int n = end - beg + 1;
      return decode_centered_minimal(n, bs);
    }
    assert(a<=mid && mid<=b);
    auto bit = bs.read();
    write(bit);
    if (bit == 0)
      b = floor(mid);
    else if (bit == 1)
      a = ceil(mid);
    else
      assert(false);
    first = false;
  }
}

/++ v is from 0 to n-1 +/
void encode_centered_minimal(uint v, uint n, ref BitStream bs) {
  assert(n > 0);
  assert(v < n);
  import std.stdio;
  int l1 = bsr(n);
  int l2 = ((1<<l1)==n) ? l1 : l1+1;
  int d = (1<<l2) - n;
  int m = (n-d) / 2;
  if (v < m) {
    bool print = false;
    v = bit_reverse(v);
    v >>= v.sizeof*8 - l2;
    bs.write(v, l2);
    //writefln("%04b", v);
  }
  else if (v >= m+d) {
    v = bit_reverse(v-d);
    v >>= v.sizeof*8 - l2;
    bs.write(v, l2);
    //writefln("%04b", v-d);
  }
  else { // middle
    v = bit_reverse(v);
    v >>= v.sizeof*8 - l1;
    bs.write(v, l1);
    //writefln("%03b", v);
  }
}

int decode_centered_minimal(uint n, ref BitStream bs) {
  assert(n > 0);
  import std.stdio;
  int l1 = bsr(n);
  int l2 = ((1<<l1)==n) ? l1 : l1+1;
  uint d = (1<<l2) - n;
  uint m = (n-d) / 2;
  bs.refill(); // TODO: minimize the number of refill
  uint v = cast(int)bs.peek(l2);
  v <<= v.sizeof*8 - l2;
  v = bit_reverse(v);
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

/++ For debugging purposes +/
void encode_array(int N, int[] nums, ref BitStream bs) {
  import std.stdio;
  float m = float(N) / 2; // mean
  float s = sqrt(float(N)) / 2; // standard deviation
  bs.init_write(100000000); // 100 MB // TODO
  foreach (n; nums) {
    //encode_range(m, s, 0, N, n, bs); // TODO: enable this
    //encode_centered_minimal(n, N+1, bs); // uniform
  }
  //int N = cast(int)positions.length;
  bs.flush();
}

void encode(T)(KdTree!(T, Root) tree, ref BitStream bs, ref Coder coder) {
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
      encode_range(m, s, 0, N, n, table, bs, coder);
      //encode_centered_minimal(n, N+1, bs); // uniform
      enc_file.writeln(n);
      if (parent.left_)
        traverse_encode(parent.left_, bs);
      if (parent.right_)
        traverse_encode(parent.right_, bs);
    }
  }
  auto table =  create_binomial_table(cutoff);
  /* pre-compute and store a table of inverse gaussian(0,1) cdf */
  bs.init_write(100000000); // 100 MB // TODO
  coder.init_write(100000000); // TODO
  //int N = cast(int)positions.length;
  int N = tree.end_ - tree.begin_;
  bs.write(N, 32);
  File enc_file = File("encode.txt", "w");
  traverse_encode(tree, bs);
  bs.flush();
  coder.encode_finalize();
  writeln("--------------");
}

void decode(ref BitStream bs, ref Coder coder) {
  import std.stdio;
  void traverse_decode(T)(int N, ref BitStream bs) {
    if (N == 1) { // leaf level
      // do nothing
    }
    else { // non leaf
      float m = float(N) / 2; // mean
      float s = sqrt(float(N)) / 2; // standard deviation
      int n = decode_range(m, s, 0, N, table, bs, coder);
      assert(n>=0 && n<=N);
      //int n = decode_centered_minimal(N+1, bs); // uniform
      dec_file.writeln(n);
      if (n > 0)
        traverse_decode!T(n, bs);
      if (n < N)
        traverse_decode!T(N-n, bs);
    }
  }

  auto table =  create_binomial_table(cutoff);
  bs.init_read();
  coder.init_read();
  int N = cast(int)bs.read(32);
  File dec_file = File("decode.txt", "w");
  dec_file.writeln(N);
  traverse_decode!int(N, bs);
}

/++ Create a probability table for small N +/
uint[][] create_binomial_table(int N) {
  auto table = pascal_triangle(N);
  for (int n = 0; n <= N; ++n) {
    for (int k = 1; k <= n; ++k)
      table[n][k] += table[n][k-1];
  }
  return table;
}
