module arithmetic_coder;

import bit_stream;
import std.conv;
import std.range;
import std.stdio;

// 64 bits = 33 (code value) + 31 (frequency) bits

struct Prob(FreqType) {
  FreqType low; // from 0 to count
  FreqType high; // from 0 to count
  FreqType count;
}

mixin template ModelConfigs(CodeVal_, int CodeValBits, int FreqBits) {
  alias CodeVal = CodeVal_;
  static const CodeVal Max_code_ = (CodeVal(1) << CodeValBits) - 1;
  static const CodeVal One_fourth_ = CodeVal(1) << (CodeValBits - 2);
  static const CodeVal One_half_ = 2 * One_fourth_;
  static const CodeVal Three_fourths_ = 3 * One_fourth_;
  static const int Code_val_bits_ = CodeValBits;
  static const int Freq_bits_ = FreqBits;
}

/++ Store probability table of characters from 'a' to 'z' +/
struct CharModel(CodeVal, int CodeValBits, int FreqBits)
if (FreqBits+2<=CodeValBits && CodeVal.sizeof*8>=CodeValBits+FreqBits) {
  mixin ModelConfigs!(CodeVal, CodeValBits, FreqBits);
  CodeVal[128] cdf_table_;
  CodeVal count_ = 0;

  /++ Get probability of a symbol s +/
  Prob!CodeVal get_prob(int s) {
    assert(0<=s && s <=127);
    if (s == 0)
      return Prob!CodeVal(0, cdf_table_[s], count_);
    return Prob!CodeVal(cdf_table_[s-1], cdf_table_[s], count_);
  }

  /++ Get the symbol with a given code value +/
  Prob!CodeVal get_val(CodeVal v, ref int s) {
    s = 0;
    for (; s<cdf_table_.length && cdf_table_[s]<=v; ++s) {}
    if (s == 0)
      return Prob!CodeVal(0, cdf_table_[s], count_);
    return Prob!CodeVal(cdf_table_[s-1], cdf_table_[s], count_);
  }

  /++  +/
  CodeVal get_count() {
    return count_;
  }

  /++ Collect probabilities from a string +/
  void collect_probs(R)(R s) {
    // TODO: deal with overflow
    count_ = 0;
    foreach (c; s) {
      if (0<=c && c<=127) {
        ++count_;
        ++cdf_table_[c];
      }
    }
    for (int i = 1; i < cdf_table_.length; ++i)
      cdf_table_[i] += cdf_table_[i-1];
  }
}

struct ArithmeticCoder(Model) {
  Model model_;
  alias CodeVal = model_.CodeVal;
  CodeVal low_ = 0;
  CodeVal high_ = model_.Max_code_;
  CodeVal val_ = 0;
  int pending_bits_ = 0;
  BitStream bit_stream_;

  void set_model(Model m) {
    model_ = m;
  }

  /++ Encode an entire array of non-negative integers +/
  void encode(T)(T[] s) {
    encode_init(s);
    for (size_t i = 0; i < s.length; ++i) {
      encode(s[i]);
      encode_renormalize();
    }
    encode_finalize();
  }

  /++ Encode an array of integers +/
  void encode_init(T)(T[] s) {
    low_ = 0;
    high_ = model_.Max_code_;
    pending_bits_ = 0;
    bit_stream_.init_write(s.length*int.sizeof);
    bit_stream_.write(s.length, 64-7); // write the number of symbols in the beginning of the stream
  }

  /++ Init the coder, given the size of its stream in bytes +/
  void encode_init(int bytes) {
    low_ = 0;
    high_ = model_.Max_code_;
    pending_bits_ = 0;
    bit_stream_.init_write(bytes);
  }

  /++ Make sure low <= val <= high at the end of the stream +/
  void encode_finalize() {
    ++pending_bits_;
    if (low_ < model_.One_fourth_)
      put_bits_plus_pending(0);
    else
      put_bits_plus_pending(1);
    bit_stream_.flush();
  }

  /++ Encode a single symbol +/
  void encode(int s) {
    auto p = model_.get_prob(s);
    encode(p);
  }

  /++ Encode a single symbol, but with a probability +/
  void encode(Prob!(model_.CodeVal) p) {
    model_.CodeVal range = high_ - low_ + 1;
    high_ = low_ + (range*p.high/p.count) - 1; // the -1 makes sure new high_ <= old high_ (== happens when p.high==p.count)
    low_ = low_ + (range*p.low/p.count);
  }

  void encode_renormalize() {
    while (true) {
      if (high_ < model_.One_half_)
        put_bits_plus_pending(0);
      else if (low_ >= model_.One_half_)
        put_bits_plus_pending(1);
      else if (low_ >= model_.One_fourth_ && high_ < model_.Three_fourths_) {
        ++pending_bits_;
        low_ -= model_.One_fourth_;
        high_ -= model_.One_fourth_;
      }
      else
        break;
      high_ <<= 1;
      ++high_; // shift in a 1-bit on the right
      high_ &= model_.Max_code_; // remove the already shifted bits on the left
      low_ <<= 1;
      low_ &= model_.Max_code_; // remove the already shifted bits on the left
    }
  }

  void put_bits_plus_pending(bool bit) {
    bit_stream_.write(bit); // TODO: optimize?
    bit_stream_.repeated_write(!bit, pending_bits_);
    pending_bits_ = 0;
  }

  /++ Decode an entire bit stream +/
  void decode(T)(ref T[] output) {
    ulong nsymbols = decode_init();
    output.length = nsymbols;
    for (size_t i = 0; i < output.length; ++i) {
      int s = decode();
      output[i] = to!T(s);
      decode_renormalize();
    }
  }

  /++ Note that here we store the number of symbols at the beginning of the stream +/
  ulong decode_init() {
    high_ = model_.Max_code_;
    low_ = 0;
    val_ = 0;
    bit_stream_.init_read();
    ulong nsymbols = bit_stream_.read(64-7); // read the number of symbols
    for (int i = 0; i < model_.Code_val_bits_; ++i) {
      val_ <<= 1;
      auto b = bit_stream_.read() > 0;
      val_ += b;
    }
    return nsymbols;
  }

  /++ Decode a single symbol +/
  int decode() {
    CodeVal range = high_ - low_ + 1;
    CodeVal scaled_val = ((val_-low_+1) * model_.get_count() - 1) / range;
    int s;
    auto p = model_.get_val(scaled_val, s);
    high_ = low_ + (range*p.high) / p.count - 1;
    low_ = low_ + (range*p.low) / p.count;
    return s;
  }

  void decode_renormalize() {
    while (true) {
      if (high_ < model_.One_half_) {
        // do nothing
      }
      else if (low_ >= model_.One_half_) {
        val_ -= model_.One_half_;
        low_ -= model_.One_half_;
        high_ -= model_.One_half_;
      }
      else if (low_ >= model_.One_fourth_ && high_ < model_.Three_fourths_) {
        val_ -= model_.One_fourth_;
        low_ -= model_.One_fourth_;
        high_ -= model_.One_fourth_;
      }
      else
        break;
      low_ <<= 1;
      high_ <<= 1;
      ++high_;
      val_ <<= 1;
      auto b = bit_stream_.read();
      val_ += b;
    }
  }
}
