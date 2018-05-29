module binary_tree;

import core.bitop;
import std.array;
import std.conv;
import std.math;

/++ Binary tree stored in a contiguous array, built from applying a binary operator bottom-up +/
class BinaryTree(T, alias op) {
private:
  int nleaves_;
  int nlevels_;

public:
  T[] buf_;
  alias buf_ this;

  /+ Initialize +/
  this(T[] input...) {
    /* compute the total number of nodes in the tree */
    int n = cast(int)input.length;
    int nodes = 0;
    int k = 1;
    nlevels_ = 1;
    while (k < n) {
      nodes += k;
      k *= 2;
      ++nlevels_;
    }
    nleaves_ = k>n ? (2*n-k) : k;
    nodes += nleaves_;

    /* allocate the buffer and copy the contents */
    buf_ = new T[](nodes);
    buf_[nodes-nleaves_ .. nodes] = input[n-nleaves_ .. n];
    buf_[nodes-n .. nodes-nleaves_] = input[0 .. n-nleaves_];
  }

  /+ Return the number of levels +/
  @property int nlevels() const {
    return nlevels_;
  }

  /+ Return the level of a given index +/
  int level(int index) const {
    assert(index>=0);
    return bsr(index+1);
  }

  /+ Return the [begin, end) indices of the given level +/
  int[2] index_range(int level) const {
    int begin = pow(2, level) - 1;
    int end = level+1<nlevels_ ? begin*2+1 : begin+nleaves_;
    return [begin, end];
  }

  /+ Apply the given binary operation and build the tree from bottom up +/
  void reduce() {
    auto be = index_range(nlevels_-1);
    int begin = be[0], end = be[1];
    while (begin > 0) {
      assert((end-begin)%2 == 0);
      for (int i = begin; i < end; i += 2) {
        int p = (i-1) / 2;
        buf_[p] = op(buf_[i], buf_[i+1]);
        //mixin("buf_[p] = " ~ op ~ "(buf_[i], buf_[i+1]);");
      }
      end = begin;
      begin = (begin-1) / 2;
    }
  }

  /+ Write the tree one level per line +/
  override string toString() const {
    auto a = appender!string;
    a.reserve(128);
    for (int l = 0; l < nlevels_; ++l) {
      auto be = index_range(l);
      int begin = be[0], end = be[1];
      for (int i = begin; i < end; ++i) {
        a ~= to!string(buf_[i]);
        a ~= " ";
      }
      a ~= "\n";
    }
    return a.data;
  }
}

unittest {
  import std.stdio;
  auto t = new BinaryTree!(int, (a,b)=>a+b)([1,2,3,4,5,6,7,8,9]);
  t.reduce();
  writeln(t);
}