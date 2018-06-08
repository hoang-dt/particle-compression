module io;

import std.algorithm;
import std.array;
import std.conv;
import std.exception;
import std.file;
import std.format;
import std.json;
import std.path;
import std.stdio;
import std.variant;
import array;
import expected;
import math;

/++ Read a raw binary file +/
Expected!(T[]) read_raw(T)(const string file_name) {
  try {
    return Expected!(T[])(cast(T[])std.file.read(file_name));
  }
  catch (Exception e) {
    return Expected!(T[])(e);
  }
}

/++ Write a raw binary file +/
Expected!bool write_raw(T)(const string file_name, const T[] data) {
  try {
    std.file.write(file_name, data);
    return Expected!bool(true);
  }
  catch (Exception e) {
    return Expected!bool(e);
  }
}

/++ Write an array to a raw text file +/
Expected!bool write_text(R)(const string file_name, const R data) {
  try {
    auto file = File(file_name, "w");
    foreach (e; data) {
      file.writefln("%s", e);
    }
    return Expected!bool(true);
  }
  catch (Exception e) {
    return Expected!bool(e);
  }
}

/++ Read data from a text file to an array +/
Expected!(T[]) read_text(string format, T)(const string file_name) {
  T[] data;
  try {
    auto file = File(file_name, "r");
    T val;
    while (file.readf(format, val)) {
      data ~= val;
    }
    return Expected!(T[])(data);
  }
  catch (Exception e) {
    return Expected!(T[])(e);
  }
}

struct Dataset {
  JSONValue metadata;
  // below is a sample json metadata
  //{
  //  "file": "tacc-turbulence-256x256x256-float32.raw",
  //  "name": "tacc",
  //  "field": "turbulence",
  //  "dimensionality": 3,
  //  "dims": [
  //    256,
  //    256,
  //    256
  //  ],
  //  "dtype": "float32"
  //}
  Variant data; // store a buffer to the actual data, typically of the Array3D type
}

/++ Read a json data set, including reading the raw data file from disk +/
Expected!Dataset read_json(const string file_name) {
  import std.array : array;
  Dataset d;
  try {
    string s = readText(file_name);
    d.metadata = parseJSON(s);
    auto dims = Vec3!int(to!(int[3])(d.metadata["dims"].array.map!(a => a.integer).array));
    auto raw_path = absolutePath(d.metadata["file"].str, absolutePath(dirName(file_name)));
    d.metadata["file"] = raw_path;
    writeln(d.metadata["file"]);
    string type_case(string type_name1, string type_name2)() {
      return
        "case \"" ~ type_name1 ~ "\":" ~
        "auto raw_buf = read_raw!" ~ type_name2 ~ "(raw_path);" ~
        "d.data = new Array3D!" ~ type_name2 ~ "(dims, raw_buf.value());" ~
        "break;";
    }
    switch (d.metadata["dtype"].str) {
      mixin (type_case!("int8", "byte"));
      mixin (type_case!("uint8", "ubyte"));
      mixin (type_case!("int16", "short"));
      mixin (type_case!("uint16", "ushort"));
      mixin (type_case!("int32", "int"));
      mixin (type_case!("uint32", "uint"));
      mixin (type_case!("int64", "long"));
      mixin (type_case!("uint64", "ulong"));
      mixin (type_case!("float32", "float"));
      mixin (type_case!("float64", "double"));
      default:
        throw new Exception("dtype not supported");
    }
    return Expected!(Dataset)(d);
  }
  catch (Exception e) {
    return Expected!(Dataset)(e);
  }
}
unittest {
  read_json("flame-64x64x64-float64.json");
}

/++ Read all the points from a hex mesh file (skip the hexes) +/
Vec3!double[] read_hex_meshes(const string file_name) {
  auto file = File(file_name, "r");
  int npoints;
  char[] buf;
  file.readln(buf);
  string temp;
  buf.formattedRead!"# %s %s"(npoints, temp);
  Vec3!double[] points;
  while (true) {
    if (!file.readln(buf)) {
      break;
    }
    if (buf[0]=='#') {
      continue;
    }
    else if (buf[0] == 'v') {
      double x, y, z;
      buf.formattedRead!"v %s %s %s"(x, y, z);
      points ~= Vec3!double(x, y, z);
    }
    else if (points.length != npoints) {
      continue;
    }
    else {
      break;
    }
  }
  enforce(points.length == npoints);
  return points;
}
