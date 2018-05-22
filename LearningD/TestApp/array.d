module array;

import std.algorithm.comparison;
import math;

/++ A 3D array (grid) elements of which are stored consecutively in memory +/
class Array3D(E) {
public:
  Vec3!int n_;
  alias dims = n_;
  E[] buf_;
  alias buf_ this;

  this(Vec3!int n, E[] buf = []) {
    n_ = n;
    if (buf.length == 0) {
      buf_ = new E[](product(n));
    }
    else {
      assert(buf.length >= product(n));
      buf_ = buf;
    }
  }

  override bool opEquals(const Object o) const {
    auto rhs = cast(Array3D!E)o;
    return (dims==rhs.dims) && (equal(buf_, rhs.buf_));
  }

  @property Array3D!E dup() const {
    auto copy = new Array3D!E(n_);
    copy.buf_[0 .. buf_.length] = buf_[0 .. buf_.length];
    return copy;
  }

  /+ Index with no bound checking +/
  ref E opIndex(int i, int j=0, int k=0) {
    return buf_[xyz2i(n_, Vec3!int(i,j,k))];
  }

  /+ Index with no bound checking (const) +/
  E opIndex(int i, int j=0, int k=0) const {
    return buf_[xyz2i(n_, Vec3!int(i,j,k))];
  }

  /+ Linear index +/
  ref E opIndex(long i) {
    return buf_[i];
  }

  /+ Linear index (const) +/
  E opIndex(long i) const {
    return buf_[i];
  }

  /+ Index with bound checking +/
  ref E at(int i, int j=0, int k=0)
  in { assert(i>=0&&i<n_.x && j>=0&&j<n_.y && k>=0&&k<n_.z); }
  body {
    return buf_[xyz2i(n_, Vec3!int(i,j,k))];
  }

  /+ Index with bound checking (const) +/
  E at(int i, int j=0, int k=0) const
  in { assert(i>=0&&i<n_.x && j>=0&&j<n_.y && k>=0&&k<n_.z); }
  body {
    return buf_[xyz2i(n_, Vec3!int(i,j,k))];
  }

  /+ Index with zero-padded out-of-range elements +/
  E at_zero_pad(int i, int j=0, int k=0) const {
    if (i<0||i>=n_.x || j<0||j>=n_.y || k<0||k>=n_.z) {
      return E.init;
    }
    return buf_[xyz2i(n_, Vec3!int(i,j,k))];
  }
}

unittest {
  import std.conv : to;
  import std.range : iota;
  import std.stdio : writeln;
  import math : Vec3;
  auto a = new Array3D!int(Vec3!int(2, 2, 2));
  foreach (i; iota(0, a.length)) {
    a[i] = to!int(i);
  }
  auto b = a.dup;
  a[1, 1, 1] = -1;
  auto v = a.at_zero_pad(-5, -2, 0);
  // TODO: add proper asserts()
  writeln(a);
  writeln(b);
  writeln(v);
}
