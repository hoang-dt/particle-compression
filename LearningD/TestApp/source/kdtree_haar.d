import std.algorithm;
import std.array;
import std.math;
import std.typecons;
import circular_queue;
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
    int nlevels_;
  }

  static if (R == Root) {
    void build(string order)(Vec3!T[] points) {
      order_ = order;
      points_ = points;
      nlevels_ = build_helper!(order)(this, 0, cast(int)points.length, 0);
    }

    void haar_transform() {
      haar_transform_helper(this, 0, cast(int)points_.length);
    }

    void pad() {
      pad_helper(this, 0, 0, cast(int)points_.length);
    }
  }

  /++ Return the number of levels +/
  int build_helper(string order)(KdTreeHaar!T root, int begin, int end, int dim) {
    assert(begin < end);
    left_ = right_ = null;
    if (begin+1 == end) return 1; // one particle

    int d = order[dim%3] - 'x';
    int mid = (end-begin) / 2;
    assert(mid>0 && begin+mid<end);
    auto pred = delegate (Vec3!T a, Vec3!T b) { return a[d] < b[d]; };
    topN!pred(root.points_[begin..end], mid);
    left_ = new KdTreeHaar!(T, Inner)();
    int lvl = left_.build_helper!order(root, begin, begin+mid, dim+1);
    right_ = new KdTreeHaar!(T, Inner)();
    lvl = right_.build_helper!order(root, begin+mid, end, dim+1);
    return lvl + 1;
  }

  Vec3!T haar_transform_helper(KdTreeHaar!T root, int begin, int end) {
    /* for each node, we compute and store the average of the two children */
    assert(begin < end);
    if (begin+1 == end) {
      assert(right_ is null && left_ is null);
      val_ = root.points_[begin];
      return val_;
    }
    else { // inner node, recurse
      assert(left_ !is null && right_ !is null);
      int mid = (end-begin) / 2;
      Vec3!T left_val = left_.haar_transform_helper(root, begin, begin+mid);
      Vec3!T right_val = right_.haar_transform_helper(root, begin+mid, end);
      val_ = (left_val+right_val) / T(2);
      right_.val_ = right_val - val_;
      return val_;
    }
  }

  /++ "Pad" the tree at the leaf level so that the tree becomes a full binary tree +/
  void pad_helper(KdTreeHaar!T root, int level, int begin, int end) {
    assert(begin < end);
    if (begin+1 == end) {
      if (level+1 < root.nlevels_) {
        assert(level+2 == root.nlevels_);
        left_ = new KdTreeHaar!(T, Inner)();
        right_ = new KdTreeHaar!(T, Inner)();
        left_.val_ = right_.val_ = val_;
      }
    }
    else {
      assert(left_ !is null && right_ !is null);
      int mid = (end-begin) / 2;
      left_.pad_helper(root, level+1, begin, begin+mid);
      right_.pad_helper(root, level+1, begin+mid, end);
    }
  }

  /++ List the values per level +/
  void list_vals(ref Vec3!T[] vals) {
    RefAppender!(Vec3!T[]) app = appender(&vals);
    alias Node = KdTreeHaar!(T, Inner);
    enum Tag : bool { Left, Right }
    alias NodeTag = Tuple!(Node, Tag);
    CircularQueue!NodeTag q;
    q.push(tuple(this.left_, Tag.Left));
    q.push(tuple(this.right_, Tag.Right));
    while (!q.empty()) {
      auto elem = q.pop();
      if (elem[1] == Tag.Right) {
        app ~= elem[0].val_;
      }
      if (elem[0].left_ !is null) {
        assert(elem[0].right_ !is null);
        q.push(tuple(elem[0].left_, Tag.Left));
        q.push(tuple(elem[0].right_, Tag.Right));
      }
    }
  }



}
