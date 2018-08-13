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
    //int i = s - 'a';
    int i = s;
    //assert(0<=i && i<='z'-'a');
    assert(0<=i && i <=127);
    if (i == 0)
      return Prob!CodeVal(0, cdf_table_[i], count_);
    return Prob!CodeVal(cdf_table_[i-1], cdf_table_[i], count_);
  }

  /++ Get the symbol with a given code value +/
  Prob!CodeVal get_val(CodeVal v, ref int s) {
    int i = 0;
    for (; i<cdf_table_.length && cdf_table_[i]<=v; ++i) {}
    //s = 'a' + i;
    s = i;
    if (i == 0)
      return Prob!CodeVal(0, cdf_table_[i], count_);
    return Prob!CodeVal(cdf_table_[i-1], cdf_table_[i], count_);
  }

  /++ Collect probabilities from a string +/
  void collect_probs(R)(R s) {
    // TODO: deal with overflow
    count_ = 0;
    foreach (c; s) {
      //if ('a'<=c && c<='z') {
      if (0<=c && c<=127) {
        ++count_;
        //++cdf_table_[c-'a'];
        ++cdf_table_[c];
      }
    }
    for (int i = 1; i < cdf_table_.length; ++i) {
      cdf_table_[i] += cdf_table_[i-1];
    }
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
    //write("encode init        ");
    low_ = 0;
    high_ = model_.Max_code_;
    //writeln(low_, " ", high_);
    pending_bits_ = 0;
    bit_stream_.init_write(s.length*int.sizeof);
    bit_stream_.write(s.length, 64-7); // write the number of symbols in the beginning of the stream
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
    //write("encode             ");
    auto p = model_.get_prob(s);
    model_.CodeVal range = high_ - low_ + 1;
    high_ = low_ + (range*p.high/p.count) - 1;
    low_ = low_ + (range*p.low/p.count);
    //writeln(low_, " ", high_);

  }

  void encode_renormalize() {
    //write("encode renormalize ");
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
    //writeln(low_, " ", high_);
  }

  void put_bits_plus_pending(bool bit) {
    bit_stream_.write(bit); // TODO: optimize?
    write(int(bit));
    for (int i = 0; i < pending_bits_; ++i) {
      write(int(!bit));
    }
    bit_stream_.repeated_write(!bit, pending_bits_);
    pending_bits_ = 0;
  }

  /++ Decode an entire bit stream +/
  void decode(T)(ref T[] output) {
    //alias T = ElementType!R;
    writeln("-----------------------\n");
    ulong nsymbols = decode_init();
    output.length = nsymbols;
    for (size_t i = 0; i < output.length; ++i) {
      int s = decode();
      output[i] = to!T(s);
      //writeln(to!T(s));
      decode_renormalize();
    }
  }

  ulong decode_init() {
    //write("decode init        ");
    high_ = model_.Max_code_;
    low_ = 0;
    //writeln(low_, " ", high_);
    val_ = 0;
    bit_stream_.init_read();
    for (int i = 0; i < 47; ++i) {
      write(int(bit_stream_.read() > 0));
    }
    writeln("hello");
    bit_stream_.init_read();
    ulong nsymbols = bit_stream_.read(64-7); // read the number of symbols
    //ulong nsymbols = 14;
    for (int i = 0; i < model_.Code_val_bits_; ++i) {
      val_ <<= 1;
      auto b = bit_stream_.read() > 0;
      write(int(b));
      //val_ += bit_stream_.read(); // TODO: optimize?
      val_ += b;
    }
    //writeln(val_);
    return nsymbols;
  }

  /++ Decode a single symbol +/
  int decode() {
    //write("decode             ");
    CodeVal range = high_ - low_ + 1;
    CodeVal scaled_val = ((val_-low_+1) * model_.count_ - 1) / range;
    int s;
    auto p = model_.get_val(scaled_val, s);
    high_ = low_ + (range*p.high) / p.count - 1;
    low_ = low_ + (range*p.low) / p.count;
    //writeln(low_, " ", high_);
    return s;
  }

  void decode_renormalize() {
    //write("decode renormalize ");
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
      write(int(b > 0));
      //val_ += bit_stream_.read(); // TODO: optimize?
      val_ += b;
    }
    //writeln(low_, " ", high_);
  }
}
