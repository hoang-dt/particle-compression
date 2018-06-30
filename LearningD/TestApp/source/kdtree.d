module kdtree;

import std.algorithm;
import std.bitmanip;
import std.math;
import std.traits;
import math;

public enum { Root, Inner };
public enum Mode { None, Precision, Accuracy };

struct BoundingBox(T) {
  Vec3!T min;
  Vec3!T max;
}

/++ A KdTree structure to store particles in 3D +/
class KdTree(T, int R=Root)
if (isFloatingPoint!T) {
public:
  int begin_;
  int end_;
  union {
    BitArray bits_;
    struct {
      KdTree!(T, Inner) left_ = null;
      KdTree!(T, Inner) right_ = null;
    }
  }

  static if (R == Root) {
    BoundingBox!T bbox_;
    Vec3!T[] points_;
    string order_;
    int precision_;
    T accuracy_;
    Mode mode_ = Mode.None;

    /++ prec = how many bit planes to keep (max = 23 for float and 52 for double) +/
    void set_precision(int prec) {
      assert(prec>=0 && prec<T.mant_dig);
      precision_ = prec;
      mode_ = Mode.Precision;
    }
    /++ acc = absolute accuracy, minimum is 0 +/
    void set_accuracy(T acc) {
      assert(acc >= 0);
      accuracy_ = acc;
      mode_ = Mode.Accuracy;
    }

    BoundingBox!T compute_bbox(T)(Vec3!T[] points) {
      return BoundingBox!T (
        Vec3!T(minElement!"a.x"(points).x, minElement!"a.y"(points).y, minElement!"a.z"(points).z),
        Vec3!T(maxElement!"a.x"(points).x, maxElement!"a.y"(points).y, maxElement!"a.z"(points).z)
      );
    }

    void build(string order, T)(Vec3!T[] points) {
      order_ = order;
      points_ = points;
      bbox_ = compute_bbox(points);
      build_helper!(order, T)(this, bbox_, 0, cast(int)points.length, 0); // first split
    }
  }

  /++ Split a given subtree. [begin, end) +/
  // TODO: this build routine would always proceed until there is one particle left, regardless of the
  // actual tolerance
  // We may want to have a control on the number of levels to keep
  void build_helper(string order, T)(KdTree!(T, Root) root, BoundingBox!T bbox, int begin, int end, int dim) {
    assert(begin < end); // this cannot be a leaf node
    begin_ = begin;
    end_ = end;
    left_ = right_ = null;
    int d = order[dim%3] - 'x';
    double middle = (bbox.min[d]+bbox.max[d]) / 2.0;
    auto pred = delegate (Vec3!T a) { return a[d] < middle; };
    auto right = std.algorithm.sorting.partition!(pred)(root.points_[begin..end]);
    int left_size = end-begin-cast(int)right.length;
    if (left_size > 0) {
      left_ = new KdTree!(T, Inner)();
      auto bbox_left = bbox;
      bbox_left.max[d] = middle;
      if (left_size == 1) {
        left_.build_helper_leaf!order(root, bbox_left, begin, begin+left_size, dim+1);
      }
      else {
        left_.build_helper!order(root, bbox_left, begin, begin+left_size, dim+1); // recurse on the left
      }
    }
    if (begin+left_size < end) {
      right_ = new KdTree!(T, Inner)();
      auto bbox_right = bbox;
      bbox_right.min[d] = middle;
      if (begin+left_size+1 == end) {
        right_.build_helper_leaf!order(root, bbox_right, begin+left_size, end, dim+1);
      }
      else {
        right_.build_helper!order(root, bbox_right, begin+left_size, end, dim+1); // recurse on the right
      }
    }
  }

  void build_helper_leaf(string order, T)(const KdTree!(T, Root) root, BoundingBox!T bbox, int begin, int end, int dim) {
    import std.stdio;
    assert(begin+1 == end);
    begin_ = begin;
    end_ = end;
    if (root.mode_ == Mode.None) {
      return;
    }
    bool[3] stop = false;
    Vec3!T p = root.points_[begin];
    Vec3!int e;
    frexp(p.x, e.x); frexp(p.y, e.y); frexp(p.z, e.z);
    --e.x; --e.y; --e.z;
    T epsilon = ldexp(T.epsilon, T.mant_dig-1-root.precision_);
    T[3] tolerance = root.accuracy_;
    if (root.mode_ == Mode.Precision) {
      tolerance[0] = ldexp(epsilon, e[0]);
      tolerance[1] = ldexp(epsilon, e[1]);
      tolerance[2] = ldexp(epsilon, e[2]);
    }
    // TODO: check that middle == bbox.min/bbox.max
    while (!(stop[0] && stop[1] && stop[2])) {
      int d = order[dim%3] - 'x';
      auto dd = bbox.max[d] - bbox.min[d];
      if (bbox.max[d]-bbox.min[d] > tolerance[d]*2) {
        T middle = (bbox.min[d]+bbox.max[d]) / T(2.0);
        bool left = p[d] < middle;
        bits_ ~= left;
        if (left) {
          if (bbox.max[d] != middle) {
            bbox.max[d] = middle;
          }
          else {
            stop[d] = true;
          }
        }
        else { // right
          if (bbox.min[d] != middle) {
            bbox.min[d] = middle;
          }
          else {
            stop[d] = true;
          }
        }
      }
      else {
        bits_ ~= true;
        stop[d] = true;
      }
      ++dim;
    }
  }

}

/++ Build a particle array from a kdtree +/
void kdtree_to_particles(T)(const KdTree!(T, Root) root, ref Vec3!T[] points) {
  void traverse(T, int R)(const string order, const KdTree!(T, R) node, BoundingBox!T bbox, int dim, ref Vec3!T[] points) {
    if (node.end_-node.begin_ > 1) { // non-leaf
      int d = order[dim%3] - 'x';
      double middle = (bbox.min[d]+bbox.max[d]) / 2.0;
      if (node.left_ !is null) {
        auto bbox_left = bbox;
        bbox_left.max[d] = middle;
        traverse(order, node.left_, bbox_left, dim+1, points);
      }
      if (node.right_ !is null) {
        auto bbox_right = bbox;
        bbox_right.min[d] = middle;
        traverse(order, node.right_, bbox_right, dim+1, points);
      }
    }
    else { // leaf
      if (root.mode_ != Mode.None) { // further subdivide
        for (size_t i = 0; i < node.bits_.length; ++i) {
          int d = order[dim%3] - 'x';
          double middle = (bbox.min[d]+bbox.max[d]) / 2.0;
          if (node.bits_[i]) {
            bbox.max[d] = middle;
          }
          else {
            bbox.min[d] = middle;
          }
          ++dim;
        }
      }
      auto p = (bbox.min+bbox.max) / 2;
      points ~= p;
    }
  }
  traverse(root.order_, root, root.bbox_, 0, points);
}
