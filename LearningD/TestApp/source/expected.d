module expected;

import std.exception;

/* abc /+def+/ */

/++ Contains either a value or an exception +/
struct Expected(T) {
  union {
    T val_;
    Exception except_;
  }
  alias val_ this;
  bool has_exception_;

  this(T v) {
    val_ = v;
    has_exception_ = false;
  }

  this(Exception e) {
    except_ = e;
    has_exception_ = true;
  }

  this(const string msg, const string file=__FILE__, size_t line=__LINE__) {
    except_ = new Exception(msg, file, line);
    has_exception_ = true;
  }

  bool opCast() const {
    return !has_exception_;
  }

  @property Exception exception() {
    assert(has_exception_);
    return except_;
  }

  @property ref T value() {
    assert(!has_exception_);
    return val_;
  }
}
