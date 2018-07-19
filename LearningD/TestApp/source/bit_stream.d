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

  void init_write(size_t bytes) {
    stream_ = new ubyte[](bytes);
    bitptr_ = &stream_[0];
  }

  void init_read(ubyte[] stream = null) {
    assert(!stream || stream.length>0);
    if (stream) stream_ = stream;
    bitptr_ = &stream_[0];
    bitpos_ = 0;
    refill();
  }

  void rewind() {
    bitptr_ = &stream_[0];
    bitbuf_ = bitpos_ = 0;
  }

  /++ Refill our buffer +/
  void refill() {
    assert(bitpos_ <= 64);
    bitptr_ += bitpos_ >> 3; // ignore the bytes we've consumed
    bitbuf_ = *(cast(ulong*)bitptr_); // refill
    bitpos_ &= 7; // left over bits that don't make a full byte
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

  /++ Peek "count" bits +/
  ulong peek(int count) {
    assert(bitpos_+count <= 64);
    assert(count>=0 && count<=64-7);
    ulong remaining = bitbuf_ >> bitpos_; // shift out the bits we've consumed
    return remaining & masks_[count]; // return the bottom count bits
  }

  /++ Put "count" bits into the buffer +/
  void put(ulong n, int count) {
    assert(bitpos_+count <= 64);
    assert(count>=0 && count<=64-7);
    bitbuf_ |= (n & masks_[count]) << bitpos_;
    bitpos_ += count;
  }

  /++ Consume "count" bits +/
  void consume(int count) {
    assert(count>=0 && count<=64-7);
    bitpos_ += count;
  }

  /++ Extract "count" bits from the stream_ +/
  ulong read(int count=1) {
    assert(count>=0 && count<=64-7);
    if (count+bitpos_ > 64) refill();
    ulong result = peek(count);
    bitpos_ += count;
    return result;
  }

  /++ Write "count" bits into the stream_ +/
  void write(ulong n, int count=1) {
    assert(count>=0 && count<=64-7);
    if (count+bitpos_ > 64) flush();
    put(n, count);
  }
}
