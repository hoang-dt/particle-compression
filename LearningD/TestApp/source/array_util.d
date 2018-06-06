/++
  Functions that work on array(s).
+/

module array_util;

import std.algorithm, std.algorithm.searching;
import std.math;
import std.range;
import std.traits;

/++ Compute the squared error between two arrays +/
double squared_error(T)(const T[] f1, const T[] f2) {
  assert(f1.length == f2.length);
  double err = 0;
  for (size_t i = 0; i < f1.length; ++i) {
    double diff = cast(double)(f1[i]-f2[i]);
    err += diff * diff;
  }
  return err;
}

unittest {
  assert(squared_error([ 1, 2, 3 ], [ 1, 2, 3 ]) == 0);
  assert(squared_error([ 1, 2, 3 ], [ 1, 2, 4 ]) == 1);
}

/++ Quantize an array of floating-point values to [-2^bits, 2^bits]
    Return: the exponent of the value with maximum magnitude +/
int quantize(F, I)(const F[] f, int bits, I[] g)
if (isFloatingPoint!F && isIntegral!I && isSigned!I) {
  assert(f.length == g.length);
  assert(bits > 0);
  assert(8*(I.sizeof) >= bits+1);

  auto max = f.maxElement!"std.math.abs(a)";
  int emax;
  frexp(abs(max), emax);
  auto s = ldexp(1.0, bits-emax);
  for (size_t i = 0; i < f.length; ++i) {
    g[i] = cast(I)(s*f[i]);
  }
  return emax;
}

void quantize_midtread(F, I)(const F[] f, int bits, I[] g, bool signed=true)
if (isFloatingPoint!F && isIntegral!I && isSigned!I) {
  assert(f.length == g.length);
  assert(bits > 0);
  assert(8*(I.sizeof) >= bits+1);

  auto vmin = minElement(f);
  auto vmax = maxElement(f);
  if (vmax > vmin) {
    double scale = ((1L<<bits)-1) * (1.0/(vmax-vmin));
    import std.stdio;
    writeln(scale);
    I bias = I(1) << (bits-1);
    if (signed) {
      for (size_t i = 0; i < f.length; ++i) {
        g[i] = cast(I)((f[i]-vmin)*scale+0.5) - bias;
      }
    }
    else {
      for (size_t i = 0; i < f.length; ++i) {
        g[i] = cast(I)((f[i]-vmin)*scale+0.5);
      }
    }
  }
  else { // max is same as min
    g[] = 0;
  }
}

/++ In-place replace each value in f by its difference from the previous value
    Return: the first value +/
T take_difference(T)(T[] f) {
  assert(f.length >= 2);
  for (size_t i = f.length-1; i >= 1; --i) {
    f[i] -= f[i-1];
  }
  auto first = f[0];
  f[0] = 0;
  return first;
}

void cdf53_lift(T)(T[] f, int l) {
  int n = cast(int)f.length;
  int p = 1 << l;
  int m = (n+p-1) / p;
  if (m <= 1) {
    return;
  }
  for (int x = 1; x < m; x += 2) {
    T fleft  = f[x-1];
    T fright = x<m-1 ? f[x+1] : fleft;
    f[x] -= (fleft+fright) / 2;
  }
  for (int x = 0; x < m; x += 2) {
    T fleft  = x>0 ? f[x-1] : f[x+1];
    T fright = x<m-1 ? f[x+1] : fleft;
    f[x] += (fleft+fright) / 4;
  }
  static T[] t;
  t.length = m>>1;
  int s = (m+1) >> 1;
  for (int x = 1; x < m; x += 2) {
    t[x>>1] = f[x];
    f[x>>1] = f[x-1];
  }
  if ((m&1) != 0) {
    f[m>>1] = f[m-1];
  }
  for (int x = 0; x < (m>>1); ++x) {
    f[s+x] = t[x];
  }
}
