import core.bitop;
import std.algorithm;
import std.array;
import std.container.array;
import std.conv;
import std.datetime.stopwatch;
import std.exception;
import std.math;
import std.path;
import std.random;
import std.range;
import std.stdio;
import std.traits;
import array;
import array_util;
import binary_tree;
import circular_queue;
import expected;
import io;
import lorenzo;
import math;
import number;

/**
1. Quantize a 2D or 3D field to 15-bit unsigned integers.
2. Generate 16-bit signed integer residuals using the Lorenzo predictor.
3. Turn the signed residuals, s, into unsigned ones, r, by moving the sign bit to the LSB:  r = s < 0 ? -(2 * s + 1) : 2 * s.
4. Divide the residuals into blocks of, say, N = 256 values.
5. Compute a forest of binary trees of sums of values, with each root being the sum of N values.
6. Given the sum, S = s + t, of two children, compute and accumulate the code length L(s) = S - lg(C(S, s)).

Let's assume that we get the value, S, of the root node for free.  How does this compare to interpolative coding, where we need either floor(lg(S)) or ceil(lg(S)) bits? See Teuhola's paper for how to do truncated binary coding, where we use floor(lg(S)) bits for the "middle" symbols around S/2, as we expect those to be more common.

For step 6, see this: https://math.stackexchange.com/questions/64716/approximating-the-logarithm-of-the-binomial-coefficient.

For extra credit, see what happens if you first randomly shuffle all residuals across the entire data set.
*/
void test_1(const string[] argv) {
  import std.array : array;
  enforce(argv.length == 2, "Args: [json metadata file]");
  string json_file = argv[1];
  /* read json data set */
  auto dataset = read_json(json_file);
  enforce(dataset, dataset.exception().toString());
  string dtype = dataset.metadata["dtype"].str;
  enforce(dtype=="float64", dtype ~ " not supported");
  /* quantize to 15 bit integers */
  int bits = 14;
  auto f = dataset.data.get!(Array3D!double);
  auto fq = new Array3D!int(f.dims);
  quantize_midtread(f, bits, fq);
  /* generate 16-bit residuals using the Lorenzo predictor */
  lorenzo_predict(fq);
  /* turn signed residuals into unsigned ones */
  foreach (ref int e; fq) {
    e = sign_to_lsb(e);
  }
  /* shuffle the array */
  //auto rnd = MinstdRand0(42);
  //fq.buf_ = fq.buf_.randomShuffle(rnd);
  /* build one binary tree from each block of 256 values */
  int block_size = 256;
  long nsamples = product(f.dims);
  long nblocks = (nsamples+block_size-1) / block_size;
  BinaryTree!int[] trees = new BinaryTree!int[](nblocks);
  for (int b = 0; b < nblocks; ++b) {
    trees[b] = new BinaryTree!int(fq[b*block_size .. (b+1)*block_size]);
    trees[b].reduce();
    auto last_level = trees[b].index_range(trees[b].nlevels-1);
    enforce(trees[b][0] == sum(trees[b][last_level[0] .. last_level[1]]));
  }
  /* print the values on each level */
  //for (int b = 0; b < nblocks; ++b) {
  //  auto tree = trees[b];
  //  for (int l = 0; l < tree.nlevels; ++l) {
  //    auto be = tree.index_range(l);
  //    auto file = File(text("tree",l,".txt"), "a+");
  //    for (int i = be[0]; i < be[1]; ++i) {
  //      file.writeln(tree[i]);
  //    }
  //  }
  //}
  /* Given the sum, S=s+t, of two children, compute and accumulate the code length L(s) = S-lg(C(S, s)). */
  double code_length1 = 0;
  double code_length2 = 0;
  int[] max_per_level;
  for (int b = 0; b < nblocks; ++b) {
    const auto tree = trees[b];
    /* compute the maximum on each level */
    if (tree.nlevels > max_per_level.length) {
      max_per_level = new int[](tree.nlevels);
    }
    for (int l = 0; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      max_per_level[l] = maxElement(tree[be[0] .. be[1]]);
    }
    /* estimate the code length */
    for (int i = 1; i < tree.length; i += 2) {
      int p = (i-1) / 2; // parent
      int S = tree[p];
      int s = tree[i];
      int l = tree.level(i);
      int max = max_per_level[l];
      if (S != 0) {
        if (max == 0) {
          code_length1 += 1;
        }
        else {
          code_length1 += max - log2_C_n_m_sterling(max, (s+max+1)/2);
          //code_length1 += log2(max);
        }
        code_length2 += log2(S);
      }
    }
  }
  writeln(code_length1);
  writeln(code_length2);
}

// FINDING: we cannot use the actual min/max on each level as bounds. It is better to just multiply the min/max from previous level by 2.
void test_2(const string[] argv) {
  enforce(argv.length == 2, "Args: [residual text file]");
  auto residuals = read_text!("%d\n", int)(argv[1]).value;

  auto min_val = minElement(residuals);
  auto max_val = maxElement(residuals);
  if (abs(max_val)>abs(min_val)) {
    min_val = -max_val;
  }
  else {
    max_val = -min_val;
  }
  //max_val = 2581*8;
  //min_val = -max_val;
  auto mid_val = (min_val+max_val) / 2;
  double code_length1 = 0;
  double code_length2 = 0;
  int range = max_val - min_val;
  double[] g = new double[](residuals.length);
  for (int i = 0; i < residuals.length; ++i) {
    auto L = log2_C_n_m(range, (residuals[i]-min_val));
    g[i] = L;
    code_length1 += range - L;
    code_length2 += log2(range);
  }
  //write_text("out.txt", g);
  writeln(mode(residuals));
  writeln(code_length1);
  writeln(code_length2);
  int a = 0;
}

int main(const string[] argv) {
  try {
    //test_1(argv);
    test_2(argv);
  }
  catch (Exception e) {
    writeln(e);
    return 1;
  }
  return 0;
}
