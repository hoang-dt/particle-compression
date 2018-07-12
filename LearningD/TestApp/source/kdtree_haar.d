import std.algorithm;
import std.array;
import std.math;
import std.typecons;
import circular_queue;
import math;

public enum { Root, Inner }

public enum SubbandType : char { L, H, LL, LH, HL, HH, LLL, LLH, LHL, LHH, HLL, HLH, HHL, HHH, INVALID };
alias Subband(T) = Tuple!(SubbandType, Vec3!T)[];

class KdTreeHaar(T, int R=Root) {
public:
  KdTreeHaar!(T, Inner) left_ = null;
  KdTreeHaar!(T, Inner) right_ = null;
  Vec3!T val_;
  SubbandType subband_type_ = SubbandType.INVALID;

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

    void zero_out_levels(int nlevels) {
      assert(nlevels >= 0);
      zero_out_levels_helper(this, nlevels, 0);
    }

    void organize_subbands(ref Subband!T[int] items) {
      organize_subbands_helper(this, 0, items);
    }
  }

  void to_particles_helper(int begin, int end, ref Vec3!T[] points) {
    if (end-begin > 1) { // non-leaf
      int mid = (end-begin) / 2;
      left_.to_particles_helper(begin, begin+mid, points);
      right_.to_particles_helper(begin+mid, end, points);
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
      subband_type_ = SubbandType.L;
      right_.subband_type_ = SubbandType.H;

      int d = root.nlevels_ - level;
      if (d%3 != 2) {
        // left-right ~ right-right
        assert(left_.right_ && right_.right_);
        haar(left_.right_.val_, right_.right_.val_);
        subband_type_ = SubbandType.LL;
        right_.subband_type_ = SubbandType.LH;
        left_.right_.subband_type_ = SubbandType.HL;
        right_.right_.subband_type_ = SubbandType.HH;
      }
      if (d%3 == 1) {
        // left-left-right ~ right-left-right
        assert(left_.left_.right_ && right_.left_.right_);
        haar(left_.left_.right_.val_, right_.left_.right_.val_);
        // left-right-right ~ right-right-right
        assert(left_.right_.right_ && right_.right_.right_);
        haar(left_.right_.right_.val_, right_.right_.right_.val_);
        subband_type_ = SubbandType.LLL;
        right_.subband_type_ = SubbandType.LLH;
        left_.right_.subband_type_ = SubbandType.LHL;
        right_.right_.subband_type_ = SubbandType.LHH;
        left_.left_.right_.subband_type_ = SubbandType.HLL;
        left_.right_.right_.subband_type_ = SubbandType.HHL;
        right_.left_.right_.subband_type_ = SubbandType.HLH;
        right_.right_.right_.subband_type_ = SubbandType.HHH;
      }
    }
  }

  /++ Invert the Haar process +/
  void invert_haar_transform_helper(KdTreeHaar!T root, int level, int begin, int end) {
    void invert_haar(ref Vec3!T left, ref Vec3!T right) {
      right = (left+right);
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
      left_.invert_haar_transform_helper(root, level+1, begin, begin+mid);
      right_.invert_haar_transform_helper(root, level+1, begin+mid, end);
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

  /++ Set a number of finer levels to 0 +/
  void zero_out_levels_helper(KdTreeHaar!T root, int nzero_levels, int level) {
    if (level+nzero_levels >= root.nlevels_)
      val_ = 0;
    if (left_) {
      left_.zero_out_levels_helper(root, nzero_levels, level+1);
      right_.zero_out_levels_helper(root, nzero_levels, level+1);
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

  void organize_subbands_helper(KdTreeHaar!T root, int level, ref Subband!T[int] items) {
    if (left_ is null) {
      assert(level == root.nlevels_-1);
    }
    else { // inner node, recurse
      assert(left_ && right_);
      items[level+1] ~= tuple(right_.subband_type_, right_.val_);
      left_.organize_subbands_helper(root, level+1, items);
      right_.organize_subbands_helper(root, level+1, items);
    }
  }
}
