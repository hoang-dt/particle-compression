module number;

import std.traits;

template Negabinary(T) {
  static if      (is(T==byte ) || is(T==ubyte )) ubyte  mask = 0xaa;
  else static if (is(T==short) || is(T==ushort)) ushort mask = 0xaaaa;
  else static if (is(T==int  ) || is(T==uint  )) uint   mask = 0xaaaaaaaa;
  else static if (is(T==long ) || is(T==ulong )) ulong  mask = 0xaaaaaaaaaaaaaaaa;
  else static assert(false, "Type " ~ T.stringof ~ " does not have a negabinary mask");
}

unittest {
  import std.stdio : writeln, writefln;
  writeln(ulong.sizeof);
  writefln("%b", Negabinary!ulong.mask);
}

/++ Convert a number to negabinary +/
Unsigned!T to_negabinary(T)(T v)
if (isIntegral!T) {
  auto mask = Negabinary!T.mask;
  return cast(Unsigned!T)((v+mask)^mask);
}

/++ Move the sign bit to the LSB: r = s<0 ? -(2*s+1) : 2*s +/
T sign_to_lsb(T)(T s)
if (isSigned!T) {
  return s<0 ? -(2*s+1) : 2*s;
  import std.math;
  //return std.math.abs(s);
}
