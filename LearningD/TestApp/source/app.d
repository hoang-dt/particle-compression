import core.bitop;
import std.algorithm;
import std.array;
import std.container.array;
import std.conv;
import std.datetime.stopwatch;
import std.exception;
import std.file;
import std.math;
import std.outbuffer;
import std.path;
import std.random;
import std.range;
import std.stdio;
import std.traits;
import dstats;
import array;
import array_util;
import binary_tree;
import circular_queue;
import expected;
import io;
import lorenzo;
import math;
import number;
import stats;

Array3D!int read_and_process_array(const string[] argv) {
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
  auto rnd = MinstdRand0(42);
  fq.buf_ = fq.buf_.randomShuffle(rnd);
  return fq;
}

BinaryTree!int[] build_binary_trees(int block_size, int nblocks, Array3D!int fq) {
  BinaryTree!int[] trees = new BinaryTree!int[](nblocks);
  for (int b = 0; b < nblocks; ++b) {
    trees[b] = new BinaryTree!int(fq[b*block_size .. (b+1)*block_size]);
    trees[b].reduce();
    auto last_level = trees[b].index_range(trees[b].nlevels-1);
    enforce(trees[b][0] == std.algorithm.sum(trees[b][last_level[0] .. last_level[1]]));
  }
  return trees;
}

void print_each_level(int nblocks, BinaryTree!int[] trees) {
  foreach (string name; dirEntries(".", "tree*.txt", SpanMode.shallow)) {
    remove(name);
  }
  OutBuffer[string] bufs;
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 0; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      string name = text("tree", l, ".txt");
      OutBuffer buf = bufs.get(name, null);
      if (!buf) {
        bufs[name] = new OutBuffer();
        buf = bufs[name];
      }
      for (int i = be[0]; i < be[1]; ++i) {
        buf.writef("%s\n", tree[i]);
      }
    }
  }
  foreach (data; bufs.byKeyValue()) {
    std.file.write(data.key, data.value.toBytes());
  }
}

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
  writeln("Test 1");
  auto fq = read_and_process_array(argv);
  /* build one binary tree from each block of 256 values */
  int block_size = 256;
  int nsamples = cast(int)product(fq.dims);
  int nblocks = (nsamples+block_size-1) / block_size;
  auto trees = build_binary_trees(block_size, nblocks, fq);
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
/+ Compute the code length of one level of the tree of residuals +/
void test_2(const string[] argv) {
  writeln("Test 2");
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
}

/+ Generate a binomial distribution and compute the code length of the sum +/
// TODO: generate the array using exponential/Laplace distributions
void test_3() {
  /* Generate binomial-distributed values between 0 and A, by approximating it with a Gaussian with
  muy = A/2 and sigma_squared = A/4 */
  int A = 1024;
  double sigma = sqrt(cast(double)A/4); // sigma
  double m = A / 2;
  int[] arr;
  int nvals = 2048; // generate nvals values
  for (int i = 0; (i<nvals) || (i>=nvals && is_odd(arr.length)) ; ++i) {
    auto v = rNormal(m, sigma);
    if (v>=0 && v<A) {
      arr ~= cast(int)v;
    }
  }
  int bits = 10;
  /* Compute the sum for every pair of numbers */
  int[] sum = new int[](arr.length/2);
  for (int i = 0; i < sum.length; ++i) {
    sum[i] = arr[2*i] + arr[2*i+1];
  }

  /* Approximate the code length for the leaf level using four methods:
    - assume uniform distribution
    - assume uniform distribution under each parent
    - assume binomial distribution on the leaf level
    - assume binomial distribution on the leaf level, conditioned on the parent level
    - assume binomial under each parent */
  double code_length1 = 0;
  double code_length2 = 0;
  double code_length3 = 0;
  double code_length4 = 0;
  double code_length5 = 0;
  for (int i = 0; i < arr.length; i += 2) {
    int s = arr[i];
    int p = i / 2;
    int S = sum[p];
    code_length1 += bits;
    code_length2 += log2(S);
    code_length3 += A - log2_C_n_m(A, s);
    code_length4 += S - log2_C_n_m(S, s);
    double M = 0.5 * (erf((S-m)/sqrt(2*sigma*sigma)) - erf((0-m)/sqrt(2*sigma*sigma)));
    code_length5 += A + log2(M) - log2_C_n_m(A, s);
  }
  writefln("uniform distribution = %s", code_length1);
  writefln("uniform distribution under each parent = %s", code_length2);
  writefln("binomial distribution on leaf level = %s", code_length3);
  writefln("binomial distribution under each parent = %s", code_length4);
  writefln("code length 5 = %s", code_length5);
  write_text("out.txt", sum);
}
/++ Print distributions of values under the same parent value on each level +/
// TODO: for the last level, take the nth largest sum instead of the first or second
// TODO: plot the distribution for the max operator
void test_4(const string[] argv) {
  import std.array : array;
  writeln("Test 4");
  auto fq = read_and_process_array(argv);
  /* collect statistic */
  int block_size = 256;
  int nsamples = cast(int)product(fq.dims);
  int nblocks = (nsamples+block_size-1) / block_size;
  auto trees = build_binary_trees(block_size, nblocks, fq);
  alias map = int[int];
  auto counts = new map[](trees[0].nlevels); // one map per level
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 0; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      for (int i = be[0]; i < be[1]; ++i) {
        int v = tree[i];
        ++counts[l][v];
      }
    }
  }

  /* compute one mode for each level */
  int[] modes = new int[](counts.length);
  for (int l = 0; l+1 < modes.length; ++l) {
    auto r = counts[l].byValue.array;
    if (l+1 == modes.length-1) {
      topN!"a>b"(r, 50);
      modes[l] = r[50];
    }
    else {
      modes[l] = r.reduce!max;
    }

    //modes[l] = second_largest(counts[l].byValue);
    writeln(modes[l]);
  }
  int[][] output = new int[][](modes.length); // one array per level, of samples whose parent is equal to the mode of the previous level
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 0; l < tree.nlevels; ++l) {
      if (l > 0) {
        int m = counts[l-1].byKey.filter!(k => counts[l-1][k]==modes[l-1]).array[0];
        auto be = tree.index_range(l);
        for (int i = be[0]; i < be[1]; ++i) {
          int p = tree[(i-1)/2];
          if (p == m) {
            output[l] ~= tree[i];
          }
        }
      }
    }
  }
  for (int l = 1; l < output.length; ++l) {
    write_text(text("out",l,".txt"), output[l]);
  }
}
int main(const string[] argv) {
  double[] arr;
  for (int i = 0; i < 2048; ++i) {
    arr ~= r_exponential(1);
  }
  write_text("out.txt", arr);
  try {
    //test_1(argv);
    //test_2(argv);
    //test_3();
    test_4(argv);
  }
  catch (Exception e) {
    writeln(e);
    return 1;
  }
  return 0;
}
