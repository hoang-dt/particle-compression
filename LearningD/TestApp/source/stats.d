module stats;

import std.math;
import std.random;

/++ Generate an exponential-distributed random variable +/
double r_exponential(RGen=Random)(double t, ref RGen gen=rndGen) {
  assert(t > 0);
  double y = uniform(0, 1, gen);
  double x = -log(1-y) / t;
  return x;
}
