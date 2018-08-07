module bit_stream;


/++ Bitstream_ supports only either reading or writing, not both at the same time +/
struct BitStream {
  ubyte[] stream_; // array
  ubyte* bitptr_; // Pointer to current byte
  ulong bitbuf_ = 0; // last 64 bits we read
  int bitpos_ = 0; // how many of those bits we've consumed/written

  static ulong[65] generate_masks() {
    ulong[65] masks;
    for (int i = 0; i < 64; ++i)
      masks[i] = (1UL<<i) - 1;
    masks[64] = ~0;
    return masks;
  }

  enum static ulong[65] masks_ = generate_masks();

  void rewind() {
    bitptr_ = &stream_[0];
    bitbuf_ = bitpos_ = 0;
  }

  /* ---------------- Read functions ---------------- */
  void init_read(ubyte[] stream = null) {
    assert(!stream || stream.length>0);
    if (stream) stream_ = stream;
    bitptr_ = &stream_[0];
    bitpos_ = 0;
    refill();
  }

  /++ Refill our buffer +/
  void refill() {
    assert(bitpos_ <= 64);
    bitptr_ += bitpos_ >> 3; // ignore the bytes we've consumed
    bitbuf_ = *(cast(ulong*)bitptr_); // refill
    bitpos_ &= 7; // left over bits that don't make a full byte
  }

  /++ Peek "count" bits
  count must be at most 64-bitpos_ +/
  ulong peek(int count) {
    assert(count>=0 && bitpos_+count<=64);
    ulong remaining = bitbuf_ >> bitpos_; // shift out the bits we've consumed
    return remaining & masks_[count]; // return the bottom count bits
  }

  /++ Consume "count" bits
  count must be at most 64-7 +/
  void consume(int count) {
    assert(count>=0 && count<=64-7);
    bitpos_ += count;
  }

  /++ Extract "count" bits from the stream
  count must be at most 64-7 +/
  ulong read(int count=1) {
    assert(count>=0 && count<=64-7);
    if (count+bitpos_ > 64) refill();
    ulong result = peek(count);
    bitpos_ += count;
    return result;
  }

  /* ---------------- Write functions ---------------- */

  void init_write(size_t bytes) {
    stream_ = new ubyte[](bytes);
    bitptr_ = &stream_[0];
  }

  /++ Flush the written bits in our buffer +/
  void flush() {
    assert(bitpos_ <= 64);
    *(cast(ulong*)bitptr_) = bitbuf_;
    int bytepos = bitpos_ >> 3;
    bitbuf_ = (bitbuf_>>1) >> ((bytepos<<3)-1);
    bitptr_ += bytepos;
    bitpos_ &= 7;
  }

  /++ Put "count" bits into the buffer
  count must be at most 64-bitpos_ +/
  void put(ulong n, int count) {
    assert(count>=0 && bitpos_+count <= 64);
    bitbuf_ |= (n & masks_[count]) << bitpos_;
    bitpos_ += count;
  }

  /++ Write "count" bits into the stream
  count must be at most 64-7 +/
  void write(ulong n, int count=1) {
    assert(count>=0 && count<=64-7);
    if (count+bitpos_ > 64) flush();
    put(n, count);
  }

  /++ Write "count" bits into the stream
  count has no restriction +/
  void repeated_write(bool b, int count) {
    assert(count >= 0);
    ulong n = ~(ulong(b)-1);
    if (count <= 64-7) { // write at most 57 bits
      write(b, count);
    }
    else { // write more than 57 bits
      while (true) {
        int nbits = 64 - bitpos_;
        if (nbits <= count) {
          put(n, nbits);
          count -= nbits;
          flush();
        }
        else {
          put(n, count);
          break;
        }
      }
    }
  }
}
