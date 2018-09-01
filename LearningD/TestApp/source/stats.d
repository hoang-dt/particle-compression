module stats;

import std.algorithm;
import std.math;
import std.random;
import std.range;
import std.traits;
import number;

/++ Generate an exponential-distributed random variable +/
double r_exponential(RGen=Random)(double t, ref RGen gen=rndGen) {
  assert(t > 0);
  double y = uniform(0.0, 1.0, gen);
  double x = -log(1-y) / t;
  return x;
}

/++ Estimate the maximum likelihood of l in the exponential distribution f(x) = l*e^(-lx) +/
double estimate_exponential_parameter(R)(R vals) {
  double s = 0;
  int count = 0;
  foreach (e; vals) {
    s += e;
  }
  return cast(double)vals.length / s;
}

double ml_exponential_even(R)(R vals)
if (isIntegral!(ElementType!R)) {
  double s = 0;
  int count = 0;
  for (size_t i = 0; i < vals.length; ++i) {
    if (is_even(vals[i])) {
      s += vals[i];
      ++count;
    }
  }
  return cast(double)count / s;
}

double ml_exponential_positive(R)(R vals) {
  double s = 0;
  int count = 0;
  foreach (e; vals) {
    if (e >= 0) {
      s += e;
      ++count;
    }
  }
  return cast(double)count / s;
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

/++ Given a range of symbols, compute the probability of each symbol +/
auto compute_probabilities(T)(T[] arr) {
  double[T] probs;
  foreach (e; arr) {
    ++probs[e];
  }
  foreach (ref e; probs) {
    e /= arr.length;
  }
  return probs;
}

auto estimate_gamma_parameters(R)(R vals) {
  auto digamma = function (double a) => log(a) - 1/(2*a);
  auto digamma_deriv = function (double a) => 1/(2*a*a) + 1/a;
  double log_x_bar = 0;
  auto avg = reduce!((a,b)=>a+b)(0.0, vals) / double(vals.length);
  auto log_avg = log(avg);
  auto avg_log = reduce!((a,b)=>a+log(b))(0.0, vals) / double(vals.length);
  double a = 0.5 / (log_avg-avg_log);
  while (true) {
    double old = a;
    double one_over_a = 1/a + ((avg_log-log_avg+log(a)-digamma(a))/(a*a*(1/a-digamma_deriv(a))));
    a = 1 / one_over_a;
    if (abs(old-a) < 1e-3) {
      break;
    }
  }
  double b = avg / a;
  return [a, b];
}

/++ Compute the inverse of the error function
Source: https://people.maths.ox.ac.uk/gilesm/files/gems_erfinv.pdf +/
float erfinv(float x) {
  float w, p;
  w = -log((1.0f-x)*(1.0f+x));
  if (w < 5.000000f) {
    w = w - 2.500000f;
    p = 2.81022636e-08f;
    p = 3.43273939e-07f  + p*w;
    p = -3.5233877e-06f  + p*w;
    p = -4.39150654e-06f + p*w;
    p = 0.00021858087f   + p*w;
    p = -0.00125372503f  + p*w;
    p = -0.00417768164f  + p*w;
    p = 0.246640727f     + p*w;
    p = 1.50140941f      + p*w;
  }
  else {
    w = sqrt(w) - 3.000000f;
    p = -0.000200214257f;
    p = 0.000100950558f + p*w;
    p = 0.00134934322f  + p*w;
    p = -0.00367342844f + p*w;
    p = 0.00573950773f  + p*w;
    p = -0.0076224613f  + p*w;
    p = 0.00943887047f  + p*w;
    p = 1.00167406f     + p*w;
    p = 2.83297682f     + p*w;
  }
  return p * x;
}

/++ Compute the inverse of the cumulative normal distribution (m=mean, s=stddev) +/
float normal_inv(float z, float m, float s) {
  return m - erfinv(1-2*z)*sqrt(2.0f)*s;
}
