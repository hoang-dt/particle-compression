module stats;

import std.algorithm;
import std.math;
import std.random;

/++ Generate an exponential-distributed random variable +/
double r_exponential(RGen=Random)(double t, ref RGen gen=rndGen) {
  assert(t > 0);
  double y = uniform(0.0, 1.0, gen);
  double x = -log(1-y) / t;
  return x;
}

/++ Find the mode of an array (return a range of elements whose value is equal to the mode) +/
auto mode(T)(T[] items) {
  int[T] aa;
  foreach (item; items) {
    aa[item]++;
  }
  auto m = aa.byValue.reduce!max;
  return aa.byKey.filter!(k => aa[k]==m);
}
