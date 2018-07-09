import std.algorithm;
import math;

public enum { Root, Inner }

class KdTreeHaar(T, int R=Root) {
public:
  KdTreeHaar!(T, Inner) left_ = null;
  KdTreeHaar!(T, Inner) right_ = null;
  Vec3!T val_;

  static if (R == Root) {
    string order_;
    Vec3!T[] points_;
  }

  static if (R == Root) {
    void build(string order)(Vec3!T[] points) {
      order_ = order;
      points_ = points;
      build_helper!(order)(this, 0, cast(int)points.length, 0);
    }

    void haar_transform() {
      haar_transform_helper(this, 0, cast(int)points_.length);
    }
  }

  void build_helper(string order)(KdTreeHaar!T root, int begin, int end, int dim) {
    assert(begin < end);
    left_ = right_ = null;
    if (begin+1 == end) return; // one particle

    int d = order[dim%3] - 'x';
    int mid = (end-begin) / 2;
    assert(mid>0 && begin+mid<end);
    auto pred = delegate (Vec3!T a, Vec3!T b) { return a[d] < b[d]; };
    topN!pred(root.points_[begin..end], mid);
    left_ = new KdTreeHaar!(T, Inner)();
    left_.build_helper!order(root, begin, begin+mid, dim+1);
    right_ = new KdTreeHaar!(T, Inner)();
    right_.build_helper!order(root, begin+mid, end, dim+1);
  }

  Vec3!T haar_transform_helper(KdTreeHaar!T root, int begin, int end) {
    /* for each node, we compute and store the average of the two children */
    if (begin+1 == end) {
      assert(right_ is null && left_ is null);
      return root.points_[begin];
    }
    else { // inner node, recurse
      assert(left_ !is null && right_ !is null);
      int mid = (end-begin) / 2;
      Vec3!T left_val = haar_transform_helper(root, begin, mid);
      Vec3!T right_val = haar_transform_helper(root, begin+mid, end);
      val_ = (left_val+right_val) / T(2);
      right_.val_ = right_.val_ - val_;
      return val_;
    }
  }
}
