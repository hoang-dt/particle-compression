module lorenzo;

import array;
import math;

/++ Store the front in Lorenzo predictor +/
class Front(T) {
private:
  uint m_; // m_ = 2^k-1 for some k (& with this to compute the circular index)
  int idx_; // current index
  Vec3!int d_; // offsets
  T[] buf_; // circular buffer of the front

public:
  this(Vec3!int n) {
    d_.x = 1;
    d_.y = n.x + 1;
    d_.z = (n.x+1) * (n.y+1);
    m_ = mask(sum(d_));
    buf_ = new T[](m_+1);
  }

  /+ add n copies of sample v to front +/
  void push(T v, int n=1) {
    for (int i = 0; i < n; ++i) {
      buf_[(idx_+i) & m_] = v;
    }
    idx_ = idx_ + n;
  }

  /+ advance front to (i, j, k) relative to current sample and fill with zeros +/
  void advance(int i, int j, int k) {
    push(0, d_.x*i+d_.y*j+d_.z*k);
  }

  /+ fetch neighbor relative to current sample +/
  ref T opIndex(int i, int j, int k) {
    return buf_[(idx_-d_.x*i-d_.y*j-d_.z*k) & m_];
  }

  /+ return m = 2^k-1 >= n-1 +/
  uint mask(uint n) const {
    for (n--; n & (n+1); n |= n+1) {}
    return n;
  }
}

/++ Compute the residuals using Lorenzo predictor +/
void lorenzo_predict(T)(Array3D!T f) {
  import std.stdio;
  auto fr = new Front!T(f.dims);

  for ({int z = 0; fr.advance(0, 0, 1);} z < f.dims.z; ++z) {
    for ({int y = 0; fr.advance(0, 1, 0);} y < f.dims.y; ++y) {
      for ({int x = 0; fr.advance(1, 0, 0);} x < f.dims.x; ++x) {
        T p = fr[1,0,0] - fr[0,1,1] + fr[0,1,0] - fr[1,0,1] + fr[0,0,1] - fr[1,1,0] + fr[1,1,1]; // prediction
        T a = f[x,y,z]; // original value
        T e = a - p; // residual
        f[x,y,z] = e; // replace the original value with the residual
        fr.push(a); // push the original value to the front
      }
    }
  }
}

/++ Simple implementation of the Lorenzo predictor +/
void lorenzo_simple(T)(const Array3D!T f, Array3D!T g)
in { assert(f.dims == g.dims); }
body {
  import std.stdio;
  for (int z = 0; z < f.dims.z; ++z) {
    for (int y = 0; y < f.dims.y; ++y) {
      for (int x = 0; x < f.dims.x; ++x) {
        T p = f.at_zero_pad(x-1,y-0,z-0) - f.at_zero_pad(x-0,y-1,z-1) + f.at_zero_pad(x-0,y-1,z-0) -
          f.at_zero_pad(x-1,y-0,z-1) + f.at_zero_pad(x-0,y-0,z-1) - f.at_zero_pad(x-1,y-1,z-0) + f.at_zero_pad(x-1,y-1,z-1); // prediction
        T a = f[x,y,z]; // original value
        T e = a - p; // residual
        g[x,y,z] = e; // store the residual
      }
    }
  }
}
