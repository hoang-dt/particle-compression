module arithmetic_coder;

import bit_stream;

// 64 bits = 33 (code value) + 31 (frequency) bits

struct Prob(FreqType) {
  FreqType low;
  FreqType high;
  FreqType count;
}

struct Model(CodeValType, CodeValBits, FreqBits) {
  static const CodeValType Max_code_ = (CodeVal(1) << CodeValBits) - 1;
  static const CodeValType One_fourth_ = CodeVal(1) << (CodeValBits - 2);
  static const CodeValType One_half_ = 2 * One_fourth_;
  static const CodeValType Three_fourths_ = 3 * One_fourth_;

  Prob get_prob(int s) {
    // TODO
    return Prob();
  }

}

struct Encoder(CodeValType, CodeValBits, FredBits) {
  Model!(CodeValType, CodeValBits, FreqBits) model_;
  CodeValType low_ = 0;
  CodeValType high_ = Max_code_;
  int pending_bits_ = 0;
  BitStream bit_stream_;

  void init() {
    // TODO
  }

  void putb_bits_plus_pending(bool bit, ref int nbits) {
    // TODO
    bit_stream_.put(bit, 1);
    bit_stream_.put();
  }

  void encode(int s) {
    Prob p = model_.get_prob(s);
    CodeValType range = high_ - low_ + 1;
    high_ = low_ + (range*p.high/p.count) - 1;
    low_ = low_ + (range*p.low/p.count);
    while (true) { // renormalization
      if (high_ < model_.One_half_)
        put_bits_plus_pending(0, pending_bits);
      else if (low_ >= model_.One_half_)
        put_bits_plus_pending(1, pending_bits);
      else if (low_ >= model_.One_fourth_ && high_ < model_.Three_forths_) {
        ++pending_bits_;
        low_ -= model_.One_fourth_;
        high -= model_.One_fourth_;
      }
      else
        break;
      high_ <<= 1;
      ++high_; // shift in a 1-bit on the right
      high_ &= model_.Max_code_; // remove the already shifted bits on the left
      low_ <<= 1;
      low_ &= model_.Max_code_; // remove the already shifted bits on the left
    }
    if (s == -1) { // end of stream
      ++pending_bits_;
      if (low_ < model_.One_fourth_)
        put_bits_plus_pending(0, pending_bits);
      else
        put_bits_plus_pending(1, pending_bits);
    }
  }

  void end_encode() {
    // TODO: flush the bit stream
  }
}

