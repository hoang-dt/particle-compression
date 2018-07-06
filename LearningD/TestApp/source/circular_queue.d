module circular_queue;

import std.traits: hasIndirections;

/++ Growable circular queue +/
struct CircularQueue(T) {
  private int length_;
  private int first_, last_;
  private T[] A_ = [T.init];

  this(T[] items...) {
    foreach (x; items) {
      push(x);
    }
  }

  @property bool empty() const {
    return length_ == 0;
  }

  @property T front() {
    assert(length_ != 0);
    return A_[first_];
  }

  T opIndex(int i) {
    assert(i<length_ && i>=0);
    return A_[(first_+i) & (A_.length-1)];
  }

  void push(T item) {
    if (length_ >= A_.length) {
      size_t old_length = A_.length;
      A_.length = A_.length * 2;
      if (last_ < first_) {
        A_[old_length .. old_length+last_+1] = A_[0 .. last_+1];
        static if (hasIndirections!T) {
          A_[0 .. last_+1] = T.init; // Help for the GC.
        }
        last_ += old_length;
      }
    }
    last_ = cast(int)((last_+1) & (A_.length-1));
    A_[last_] = item;
    ++length_;
  }

  @property int length() {
    return length_;
  }

  @property T pop() {
    assert(length_ != 0);
    auto saved = A_[first_];
    static if (hasIndirections!T) {
      A_[first_] = T.init; // Help for the GC.
    }
    first_ = cast(int)((first_+1) & (A_.length-1));
    --length_;
    return saved;
  }
}
