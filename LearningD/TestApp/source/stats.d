module stats;

import std.algorithm;
import std.math;
import std.random;
import std.range;

/++ Generate an exponential-distributed random variable +/
double r_exponential(RGen=Random)(double t, ref RGen gen=rndGen) {
  assert(t > 0);
  double y = uniform(0.0, 1.0, gen);
  double x = -log(1-y) / t;
  return x;
}

/++ Estimate the maximum likelihood of l in the exponential distribution f(x) = l*e^(-lx) +/
double ml_exponential(R)(R vals) {
  double s = 0;
  foreach (e; vals) {
    s += e;
  }
  return cast(double)vals.length / s;
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

/++ Find the second largest element +/
auto second_largest(R)(R arr)
if (isInputRange!R) {
  alias T = ElementType!R;
  T first, second;

  first = second = T.min;
  foreach (e; arr) {
    if (e > first) {
      second = first;
      first = e;
    }
    else if (e>second && e!=first) {
      second = e;
    }
  }
  return second;
}
