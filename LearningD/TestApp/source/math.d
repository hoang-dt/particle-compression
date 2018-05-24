module math;

import std.math;
import std.traits;

/++ 3D vector (x, y, z) +/
struct Vec3(T)
if (isNumeric!T) {
  T x, y, z;
  this(T x_, T y_, T z_) {
    x = x_; y = y_; z = z_;
  }
  this(T[3] v) {
    x = v[0]; y = v[1], z = v[2];
  }
}

long xyz2i(T)(Vec3!T n, Vec3!T v) {
  auto vl = Vec3!long(v.x, v.y, v.z);
  return vl.z*n.x*n.y + vl.y*n.x + vl.x;
}

long product(T)(Vec3!T v) {
  auto vl = Vec3!long(v.x, v.y, v.z);
  return vl.x * vl.y * vl.z;
}

T sum(T)(Vec3!T v) {
  return v.x + v.y + v.z;
}

long product(T)(T[3] v) {
  return cast(long)v[0] * cast(long)v[1] * cast(long)v[2];
}

unittest {
  auto x = Vec3!int(1000, 2000, 4000);
  auto n = Vec3!int(2048, 2048, 2048);
  auto i = xyz2i(n, x);
  assert(i == 16781313000);
}

/++ Approixmate log2(C(n, m)) with Sterling formula +/
double log2_C_n_m_sterling(int n, int m) {
  assert(n>=m);
  if (n == m) {
    return 0;
  }
  return n*log2(n) - m*log2(m) - (n-m)*log2(n-m);
}

double log2_C_n_m(int n, int m) {
  assert(n>=m);
  if (n == m) {
    return 0;
  }
  double s = 0;
  for (double i = 1; i <= m; ++i) {
    s += log2((n+1-i)/i);
  }
  return s;
}

bool is_even(T)(T v)
if (isIntegral!T) {
  return (v&1) == 0;
}

bool is_odd(T)(T v)
if (isIntegral!T) {
  return (v&1) != 0;
}
