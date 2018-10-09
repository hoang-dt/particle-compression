module math;

import std.math;
import std.mathspecial;
import std.traits;

/++ 3D vector (x, y, z) +/
struct Vec3(T)
if (isNumeric!T) {
  T[3] p;
  alias p this;
  @property ref T x() { return p[0]; }
  @property ref T y() { return p[1]; }
  @property ref T z() { return p[2]; }
  @property T x() const { return p[0]; }
  @property T y() const { return p[1]; }
  @property T z() const { return p[2]; }

  this(T x_, T y_, T z_) {
    x = x_; y = y_; z = z_;
  }
  this(T[3] v) {
    p = v;
  }

  Vec3!T opBinary(string op)(const Vec3!T rhs) const {
    return mixin("Vec3!T(x "~op~" rhs.x, y "~op~" rhs.y, z "~op~" rhs.z)");
  }

  Vec3!T opBinary(string op)(T v) const {
    return mixin("Vec3!T(x "~op~" v, y "~op~" v, z "~op~" v)");
  }
}

struct BoundingBox(T) {
  Vec3!T min;
  Vec3!T max;
}

Vec3!T2 convert_type(T2, T1)(const Vec3!T1 p) {
  return Vec3!T2(cast(T2)p.x, cast(T2)p.y, cast(T2)p.z);
}

T2 convert_type(T2, T1)(T2 v)
if (isNumeric!T1) {
  return cast(T2)v;
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

T sqr_norm(T)(Vec3!T v) {
  return v.x*v.x + v.y*v.y + v.z*v.z;
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

int n_choose_k(int n, int k) {
  int nom = 1;
  for (int i = 0; i <= k-1; ++i)
    nom *= (n-i);
  int denom = 1;
  for (int i = 1; i <= k; ++i)
    denom *= i;
  return nom / denom;
}

/++ Approximate log2(C(n, m)) with Sterling formula +/
double log2_C_n_m_sterling(int n, int m) {
  assert(n>=m);
  if (n == m)
    return 0;
  return n*log2(n) - m*log2(m) - (n-m)*log2(n-m);
}

double log2_C_n_m(int n, int m) {
  assert(n>=m);
  if (n == m)
    return 0;
  double s = 0;
  for (double i = 1; i <= m; ++i)
    s += log2((n+1-i)/i);
  return s;
}

T square(T)(T val) {
  return val*val;
}

/++ We use erf function to achieve better accuracy for small x +/
real erfc(real x) {
  return 1 - erf(x);
}

/++ Generate the Pascal triangle +/
uint[][] pascal_triangle(int n) {
  uint[][] triangle = new uint[][](n+1);
  for (int i = 0; i <= n; ++i)
    triangle[i] = new uint[](i+1);
  for (int i = 0; i <= n; ++i) {
    triangle[i][0] = 1;
    for (int j = 1; j < i; ++j)
      triangle[i][j] = triangle[i-1][j-1] + triangle[i-1][j];
    triangle[i][i] = 1;
  }
  return triangle;
}
