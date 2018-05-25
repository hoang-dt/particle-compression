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

void quantize_midtread(F, I)(const F[] f, int bits, I[] g)
if (isFloatingPoint!F && isIntegral!I && isSigned!I) {
  assert(f.length == g.length);
  assert(bits > 0);
  assert(8*(I.sizeof) >= bits+1);

  auto vmin = minElement(f);
  auto vmax = maxElement(f);
  if (vmax > vmin) {
    double scale = ((1L<<bits)-1) * (1.0/(vmax-vmin));
    I bias = I(1) << (bits-1);
    for (size_t i = 0; i < f.length; ++i) {
      g[i] = cast(I)((f[i]-vmin)*scale+0.5) - bias;
    }
  }
  else { // max is same as min
    g[] = 0;
  }
}

