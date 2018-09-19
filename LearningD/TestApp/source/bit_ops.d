module bit_ops;

/++ Reverse the bits in the input +/
uint bit_reverse(uint a) {
  uint t;
  a = (a << 15) | (a >> 17);
  t = (a ^ (a >> 10)) & 0x003f801f;
  a = (t + (t << 10)) ^ a;
  t = (a ^ (a >>  4)) & 0x0e038421;
  a = (t + (t <<  4)) ^ a;
  t = (a ^ (a >>  2)) & 0x22488842;
  a = (t + (t <<  2)) ^ a;
  return a;
}
