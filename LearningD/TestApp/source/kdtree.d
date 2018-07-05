module kdtree;

import std.algorithm;
import std.array;
import std.bitmanip;
import std.exception;
import std.math;
import std.range;
import std.traits;
import math;

// TODO: use appender where possible

public enum { Root, Inner };
public enum Mode { None, Precision, Accuracy };

struct BoundingBox(T) {
  Vec3!T min;
  Vec3!T max;
}

// TODO: this is a hack, we should write to let the D people know about this
Range my_partition(alias predicate, Range)(Range r)
if (isRandomAccessRange!(Range) && hasLength!Range && hasSlicing!Range)
{
  //import std.algorithm.mutation : bringToFront;
  import std.functional : unaryFun;
  alias pred = unaryFun!(predicate);
  import std.algorithm.mutation : swapAt;
  // For dynamic arrays prefer index-based manipulation
  if (!r.length) return r;
  size_t lo = 0, hi = r.length - 1;
  for (;;)
  {
    for (;;)
    {
      if (lo > hi) return r[lo .. r.length];
      if (!pred(r[lo])) break;
      ++lo;
    }
    // found the left bound
    assert(lo <= hi);
    for (;;)
    {
      if (lo == hi) return r[lo .. r.length];
      if (pred(r[hi])) break;
      --hi;
    }
    // found the right bound, swap & make progress
    r.swapAt(lo++, hi--);
  }
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

    void build(string order, T, A...)(Vec3!T[] points, A a) {
      order_ = order;
      points_ = points;
      bbox_ = compute_bbox(points);
      build_helper!(order, T)(this, bbox_, 0, cast(int)points.length, 0, a); // first split
    }
  }

  /++ Split a given subtree. [begin, end) +/
  // TODO: this build routine would always proceed until there is one particle left, regardless of the
  // actual tolerance
  // We may want to have a control on the number of levels to keep
  void build_helper(string order, T, A...)(KdTree!(T, Root) root, BoundingBox!T bbox, int begin, int end, int dim, A a) {
    assert(begin < end); // this cannot be a leaf node
    begin_ = begin;
    end_ = end;
    left_ = right_ = null;
    int d = order[dim%3] - 'x';
    double middle = (bbox.min[d]+bbox.max[d]) / 2.0;
    int right_length = 0;
    static if (a.length) { // partition other arrays beside the position
      alias E = typeof(zip(root.points_, a)[0]);
      auto pred = delegate (E e) { return e[0][d] < middle; };
      auto right = my_partition!(pred)(zip(root.points_, a)[begin..end]);
    }
    else {
      auto pred = delegate (Vec3!T a) { return a[d] < middle; };
      auto right = std.algorithm.sorting.partition!(pred)(root.points_[begin..end]);
      right_length = cast(int)right.length;
    }
    int left_size = end-begin-cast(int)right.length;
    if (left_size > 0) {
      left_ = new KdTree!(T, Inner)();
      auto bbox_left = bbox;
      bbox_left.max[d] = middle;
      if (left_size == 1) {
        left_.build_helper_leaf!order(root, bbox_left, begin, begin+left_size, dim+1);
      }
      else {
        left_.build_helper!order(root, bbox_left, begin, begin+left_size, dim+1, a); // recurse on the left
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
        right_.build_helper!order(root, bbox_right, begin+left_size, end, dim+1, a); // recurse on the right
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

/++ Return true if the given node is a leaf (either null or contains 1 particle) +/
bool is_leaf(T, int R)(const KdTree!(T, R) node) {
  return ((node is null) || (node.begin_+1 == node.end_));
}

/++ Return true if the two given leaves node are of the same type (both null or both containing 1 particle) +/
bool is_same_leaf_type(T, int R1, int R2)(const KdTree!(T, R1) leaf1, const KdTree!(T, R2) leaf2) {
  if (leaf1 is null) {
    return leaf2 is null;
  }
  else if (leaf2 is null) {
    return false;
  }
  return leaf1.begin_+1==leaf1.end_ && leaf2.begin_+1==leaf2.end_;
}

/++ Compare two kdtrees and output the differences, in terms of the leaf nodes of the first tree:
  - If a leaf node is still there, output SAME
  - If a leaf node is no longer there, output GONE
  - If a leaf node now has children, output SPLIT +/
enum LeafChange { Same, Switch, Split }
void compare_kdtrees(T)(const KdTree!(T, Root) tree1, const KdTree!(T, Root) tree2, ref LeafChange[] changes) {
  import std.stdio;
  RefAppender!(LeafChange[]) app = appender(&changes);
  /++ recursive function that traverse the two trees and compare the leaf nodes +/
  void traverse(int R1, int R2)(const KdTree!(T, R1) node1, const KdTree!(T, R2) node2, RefAppender!(LeafChange[]) app) {
    if (node1 is null) {
      if (node2 is null) {
        app ~= LeafChange.Same;
      }
      else if (node2.begin_+1 == node2.end_) {
        app ~= LeafChange.Switch;
      }
      else {
        app ~= LeafChange.Split;
      }
    }
    else if (node1.begin_+1 == node1.end_) {
      if (node2 is null) {
        app ~= LeafChange.Switch;
      }
      else if (node2.begin_+1 == node2.end_) {
        app ~= LeafChange.Same;
      }
      else {
        app ~= LeafChange.Split;
      }
    }
    else {
      if (node2 is null) {
        traverse(node1.left_, node2, app);
        traverse(node1.right_, node2, app);
      }
      else if (node2.begin_+1 == node2.end_) {
        traverse(node1.left_, node2, app);
        enforce(node2.right_ is null);
        traverse(node1.right_, node2.right_, app); // node2.right_ should be null
      }
      else {
        traverse(node1.left_, node2.left_, app);
        traverse(node1.right_, node2.right_, app);
      }
    }
  }
  traverse(tree1, tree2, app);
}

/++ Count the number of leaf nodes who are not null (== the number of particles) +/
int count_leaves(T)(const KdTree!(T, Root) tree) {
  int traverse(int R)(const KdTree!(T, R) node) {
    int sum;
    if (is_leaf(node)) {
      if (node !is null) {
        return 1;
      }
    }
    else {
      sum = traverse(node.left_);
      sum += traverse(node.right_);
    }
    return sum;
  }
  return traverse(tree.left_) + traverse(tree.right_);
}
