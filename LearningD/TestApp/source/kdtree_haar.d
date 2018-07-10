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
      nlevels_ = build_helper!(order)(this, 0, cast(int)points_.length, 0);
    }

    void haar_transform() {
      haar_transform_helper(this, 0, 0, cast(int)points_.length);
    }

    void invert_haar_transform() {
      invert_haar_transform_helper(this, 0, 0, cast(int)points_.length);
    }

    void pad() {
      pad_helper(this, 0, 0, cast(int)points_.length);
    }

    void to_particles(ref Vec3!T[] points) {
      to_particles_helper(0, cast(int)points_.length, points);
    }
  }

  void to_particles_helper(int begin, int end, ref Vec3!T[] points) {
    if (end-begin > 1) { // non-leaf
      if (left_) {
        int mid = (begin+end) / 2;
        left_.to_particles_helper(begin, begin+mid, points);
        right_.to_particles_helper(begin+mid, end, points);
      }
    }
    else { // leaf
      points ~= val_;
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

  void haar_transform_helper(KdTreeHaar!T root, int level, int begin, int end) {
    void haar(ref Vec3!T left, ref Vec3!T right) {
      auto mid = (left+right) / 2;
      left = mid;
      right = right - mid;
    }
    /* for each node, we compute and store the average of the two children */
    assert(begin <= end);
    if (left_ is null) {
      assert(level == root.nlevels_-1);
      val_ = root.points_[begin];
    }
    else { // inner node, recurse
      assert(left_ !is null && right_ !is null);
      int mid = (end-begin) / 2;
      left_.haar_transform_helper(root, level+1, begin, begin+mid);
      right_.haar_transform_helper(root, level+1, begin+mid, end);
      val_ = left_.val_;
      haar(val_, right_.val_);
      int d = root.nlevels_ - level;
      if (d%3 != 2) {
        // left-right ~ right-right
        assert(left_.right_ && right_.right_);
        haar(left_.right_.val_, right_.right_.val_);
      }
      if (d%3 == 1) {
        // left-left-right ~ right-left-right
        assert(left_.left_.right_ && right_.left_.right_);
        haar(left_.left_.right_.val_, right_.left_.right_.val_);
        // left-right-right ~ right-right-right
        assert(left_.right_.right_ && right_.right_.right_);
        haar(left_.right_.right_.val_, right_.right_.right_.val_);
      }
    }
  }

  /++ Invert the Haar process +/
  void invert_haar_transform_helper(KdTreeHaar!T root, int level, int begin, int end) {
    void invert_haar(ref Vec3!T left, ref Vec3!T right) {
      right = (left+right) / 2;
      left = left * 2 - right;
    }
    assert(begin <= end);
    if (left_ is null) {
      assert(level == root.nlevels_-1);
      return;
    }
    else { // inner node
      assert(left_ && right_);
      int d = root.nlevels_ - level;
      if (d%3 == 1) {
        // left-right-right ~ right-right-right
        assert(left_.right_.right_ && right_.right_.right_);
        invert_haar(left_.right_.right_.val_, right_.right_.right_.val_);
        // left-left-right ~ right-left-right
        assert(left_.left_.right_ && right_.left_.right_);
        invert_haar(left_.left_.right_.val_, right_.left_.right_.val_);
      }
      if (d%3 != 2) {
        // left-right ~ right-right
        assert(left_.right_ && right_.right_);
        invert_haar(left_.right_.val_, right_.right_.val_);
      }
      invert_haar(val_, right_.val_);
      left_.val_ = val_;
      int mid = (end-begin) / 2;
      left_.haar_transform_helper(root, level+1, begin, begin+mid);
      right_.haar_transform_helper(root, level+1, begin+mid, end);
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
