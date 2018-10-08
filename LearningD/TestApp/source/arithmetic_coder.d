module arithmetic_coder;

import bit_stream;
import std.conv;
import std.range;
import std.stdio;

// 64 bits = 33 (code value) + 31 (frequency) bits
// TODO: Merge model with the arithmetic c

struct Prob(CountT) {
  CountT low; // from 0 to count
  CountT high; // from 0 to count
  CountT count;
}

/++ Store probability table of characters from 'a' to 'z' +/
//struct CharModel(CodeT, int CodeValBits, int FreqBits)
//if (FreqBits+2<=CodeValBits && CodeT.sizeof*8>=CodeValBits+FreqBits) {
//  mixin ModelConfigs!(CodeT, CodeValBits, FreqBits);
//  CodeT[128] cdf_table_;
//  CodeT count_ = 0;
//
//  /++ Get probability of a symbol s +/
//  Prob!CodeT get_prob(int s) {
//    assert(0<=s && s <=127);
//    if (s == 0)
//      return Prob!CodeT(0, cdf_table_[s], count_);
//    return Prob!CodeT(cdf_table_[s-1], cdf_table_[s], count_);
//  }
//
//  /++ Get the symbol with a given code value +/
//  Prob!CodeT get_val(CodeT v, ref int s) {
//    s = 0;
//    for (; s<cdf_table_.length && cdf_table_[s]<=v; ++s) {}
//    if (s == 0)
//      return Prob!CodeT(0, cdf_table_[s], count_);
//    return Prob!CodeT(cdf_table_[s-1], cdf_table_[s], count_);
//  }
//
//  /++  +/
//  CodeT get_count() {
//    return count_;
//  }
//
//  /++ Collect probabilities from a string +/
//  void collect_probs(R)(R s) {
//    // TODO: deal with overflow
//    count_ = 0;
//    foreach (c; s) {
//      if (0<=c && c<=127) {
//        ++count_;
//        ++cdf_table_[c];
//      }
//    }
//    for (int i = 1; i < cdf_table_.length; ++i)
//      cdf_table_[i] += cdf_table_[i-1];
//  }
//}

/++ Condition: CodeValBits >= CountBits+2 +/
struct ArithmeticCoder(CodeT=ulong, CountT=uint, int CodeBits=33, int CountBits=31) {
  static const CodeT CodeMax = (CodeT(1)<<CodeBits) - 1;
  static const CodeT CodeOneFourth = CodeT(1) << (CodeBits-2);
  static const CodeT CodeOneHalf = 2 * CodeOneFourth;
  static const CodeT CodeThreeFourths = 3 * CodeOneFourth;

  CodeT m_code_low, m_code_high, m_code_val;
  int m_pending_bits;
  BitStream m_bit_stream;

  /++ Init for encoding
  bytes = the size of the compressed stream in bytes +/
  void init_write(int bytes) {
    m_code_low = m_code_val = m_pending_bits = 0;
    m_code_high = CodeMax;
    m_bit_stream.init_write(bytes);
  }

  /++ Init for decoding +/
  void init_read() {
    m_code_low = m_code_val = m_pending_bits = 0;
    m_code_high = CodeMax;
    m_bit_stream.init_read();
  }

  /++ Make sure low <= val <= high at the end of the stream +/
  void encode_finalize() {
    ++m_pending_bits;
    if (m_code_low < CodeOneFourth)
      put_bits_plus_pending(0);
    else
      put_bits_plus_pending(1);
    m_bit_stream.flush();
  }

  /++ Encode a single symbol +/
  void encode(Prob!CountT p) {
    CodeT range = m_code_high - m_code_low + 1;
    m_code_high = m_code_low + (range*p.high/p.count) - 1; // the -1 makes sure new m_code_high <= old m_code_high (== happens when p.high==p.count)
    m_code_low = m_code_low + (range*p.low/p.count);
  }

  void encode_renormalize() {
    while (true) {
      if (m_code_high < CodeOneHalf)
        put_bits_plus_pending(0);
      else if (m_code_low >= CodeOneHalf)
        put_bits_plus_pending(1);
      else if (m_code_low >= CodeOneFourth && m_code_high < CodeThreeFourths) {
        ++m_pending_bits;
        m_code_low -= CodeOneFourth;
        m_code_high -= CodeOneFourth;
      }
      else
        break;
      m_code_high <<= 1;
      ++m_code_high; // shift in a 1-bit on the right
      m_code_high &= CodeMax; // remove the already shifted bits on the left
      m_code_low <<= 1;
      m_code_low &= CodeMax;
    }
  }

  private void put_bits_plus_pending(bool bit) {
    m_bit_stream.write(bit); // TODO: optimize?
    m_bit_stream.repeated_write(!bit, m_pending_bits);
    m_pending_bits = 0;
  }

  void decode_begin() {
    for (int i = 0; i < CodeBits; ++i) { // TODO: what if we read past the stream?
      m_code_val <<= 1;
      auto b = m_bit_stream.read() > 0;
      m_code_val += b;
    }
  }

  /++ Decode a single symbol and return its index in the CDF table +/
  size_t decode(in CountT[] cdf_table, CodeT count) {
    CodeT range = m_code_high - m_code_low + 1;
    CodeT v = ((m_code_val-m_code_low+1)*count-1) / range;
    size_t s = 0;
    for (; s<cdf_table.length && cdf_table[s]<=v; ++s) {}
    CountT low = s==0 ? 0 : cdf_table[s-1];
    CountT high = cdf_table[s];
    m_code_high = m_code_low + (range*high)/count - 1;
    m_code_low = m_code_low + (range*low)/count;
    return s;
  }

  void decode_renormalize(ref ArithmeticCoder c) {
    while (true) {
      if (m_code_high < CodeOneHalf) {
        // do nothing
      }
      else if (m_code_low >= CodeOneHalf) {
        m_code_val -= CodeOneHalf;
        m_code_low -= CodeOneHalf;
        m_code_high -= CodeOneHalf;
      }
      else if (m_code_low>=CodeOneFourth && m_code_high<CodeThreeFourths) {
        m_code_val -= CodeOneFourth;
        m_code_low -= CodeOneFourth;
        m_code_high -= CodeOneFourth;
      }
      else
        break;
      m_code_low <<= 1;
      m_code_high <<= 1;
      ++m_code_high;
      m_code_val <<= 1;
      m_code_val += m_bit_stream.read();
    }
  }
}
