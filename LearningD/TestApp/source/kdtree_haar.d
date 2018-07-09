import std.algorithm;
import math;

public enum { Root, Inner }

class KdTreeHaar(T, int R=Root) {
public:
  KdTreeHaar!(T, Inner) left_ = null;
  KdTreeHaar!(T, Inner) right_ = null;

  static if (R == Root) {
    string order_;
    Vec3!T[] points_;
  }

  void build(string order, T)(Vec3!T[] points) {
    order_ = order;
    points_ = points;
    build_helper!(order, T)(this, 0, cast(int)points.length, 0);
  }

  void build_helper(string order, T)(KdTreeHaar!T root, int begin, int end, int dim) {
    assert(begin < end);
    left_ = right_ = null;
    if (begin+1 == end) return; // one particle

    int d = order[dim%3] - 'x';
    int mid = (end-begin) / 2;
    topN(root.points_[begin..end], mid);
    if (mid > 0) {
      left_ = new KdTreeHaar!(T, Inner)();
      left_.build_helper!order(root, begin, begin+mid, dim+1);
    }
    if (begin+mid < end) {
      right_ = new KdTreeHaar!(T, Inner)();
      right_.build_helper!order(root, begin+mid, end, dim+1);
    }
  }
}
