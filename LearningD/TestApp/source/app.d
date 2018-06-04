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


/++ Read a raw data set and process it into 16-bit unsigned, shuffled array of residuals +/
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
    //e = sign_to_lsb(e);
  }
  /* shuffle the array */
  //auto rnd = MinstdRand0(42);
  //fq.buf_ = fq.buf_.randomShuffle(rnd);
  return fq;
}

/++ Generate an exponentially-distributed array of 16-bit unsigned integers +/
int[] generate_array_exponential(double l) {
  writeln("generate array exponential");
  double[] f;
  for (int i = 0; i < 384*384*64; ++i) {
    f ~= r_exponential(l);
  }
  int[] fq = new int[](f.length);
  int bits = 8;
  quantize(f, bits, fq);
  double lambda = ml_exponential(fq);
  writeln("lambda = ", lambda);
  //for (int i = 0; i < fq.length; ++i) {
  //  fq[i] = cast(int)f[i];
  //}
  return fq;
}

/++ Generate an exponentially-distributed array of 16-bit unsigned integers +/
double[] generate_array_exponential_double(double l) {
  double[] f;
  for (int i = 0; i < 384*384*64; ++i) {
    f ~= r_exponential(l);
  }
  //write_text("out.txt", f);
  double lambda = ml_exponential(f);
  writeln("lambda = ", lambda);
  return f;
}

BinaryTree!(T,op)[] build_binary_trees(R, alias op, T=ElementType!R)(int block_size, int nblocks, R fq) {
  BinaryTree!(T,op)[] trees = new BinaryTree!(T,op)[](nblocks);
  for (int b = 0; b < nblocks; ++b) {
    trees[b] = new BinaryTree!(T,op)(fq[b*block_size .. (b+1)*block_size]);
    trees[b].reduce();
    auto last_level = trees[b].index_range(trees[b].nlevels-1);
    //enforce(trees[b][0] == std.algorithm.sum(trees[b][last_level[0] .. last_level[1]]));
    enforce(trees[b][0] == std.algorithm.maxElement(trees[b][last_level[0] .. last_level[1]]));
  }
  return trees;
}

void print_each_level(T,F)(int nblocks, BinaryTree!(T, F)[] trees) {
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
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>a+b)(block_size, nblocks, fq);
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
  write_text("test_3_out.txt", sum);
}

/++ Print distributions of values under the same parent value on each level +/
void test_4(const string[] argv) {
  import std.array : array;
  writeln("Test 4");
  auto fq = read_and_process_array(argv);
  //auto fq = generate_array_exponential(1);
  /* collect statistic */
  int block_size = 256;
  int nsamples = cast(int)fq.length;
  int nblocks = (nsamples+block_size-1) / block_size;
  //auto trees = build_binary_trees!(typeof(fq), (a,b)=>a+b)(block_size, nblocks, fq);
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>max(a,b))(block_size, nblocks, fq);
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
    //if (l+2 >= modes.length-1) {
      topN!"a>b"(r, 7);
      modes[l] = r[7];
    //}
    //else {
      //modes[l] = r.reduce!max;
    //}

    writeln(modes[l]);
  }
  int[][] output = new int[][](modes.length); // one array per level, of samples whose parent is equal to the mode of the previous level
  int[] m = new int[](modes.length);
  for (int l = 1; l < m.length; ++l) {
    m[l] = counts[l-1].byKey.filter!(k=>counts[l-1][k]==modes[l-1]).array[0];
    writeln(m[l]);
  }
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 1; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      for (int i = be[0]; i < be[1]; i += 2) {
        int p = tree[(i-1)/2];
        //enforce(p == (tree[i] + tree[i+1]));
        if (p == m[l]) {
          //output[l] ~= tree[i]; // NOTE: this is used to plot the left child
          //output[l] ~= abs(tree[i] - tree[i+1]); // NOTE: this is used to plot |left child - right child|
          //auto diff = tree[i] - tree[i+1];
          //if (is_odd(diff)) {
            output[l] ~= tree[i] - tree[i+1]; // NOTE: this is used to plot (left child - right child)
          //}
          //if (tree[i+1] == p) {
            //output[l] ~= tree[i];
          //}
        }
      }
    }
  }
  for (int l = 1; l < output.length; ++l) {
    write_text(text("test_4_out",l,".txt"), output[l]);
  }
}

/++ Print unconditioned distributions of left-right children on each level +/
void test_5(const string[] argv) {
  import std.array : array;
  writeln("Test 5");
  auto fq = read_and_process_array(argv);
  //write_text("residuals.txt", fq);
  double lambda = ml_exponential(fq);
  //writeln("maximum likelihood exponential = ", lambda);
  auto gen = generate_array_exponential_double(lambda);
  //write_text("exponential.txt", gen);
  /* collect statistic */
  int block_size = 256;
  int nsamples = cast(int)fq.length;
  int nblocks = (nsamples+block_size-1) / block_size;
  //auto trees = build_binary_trees!(typeof(fq), (a,b)=>a+b)(block_size, nblocks, fq); // NOTE: use the sum operator
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>max(a,b))(block_size, nblocks, fq); // NOTE: use the max operator
  int[][] output = new int[][](trees[0].nlevels); // one array per level, of samples whose parent is equal to the mode of the previous level
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 1; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      for (int i = be[0]; i < be[1]; i += 2) {
        output[l] ~= tree[i] - tree[i+1]; // NOTE: this is used to plot (left child - right child)
      }
    }
  }
  for (int l = 1; l < output.length; ++l) {
    write_text(text("test_5_out",l,".txt"), output[l]);
  }
}

/++ Compute int_{a}^{b}{lambda*exp(-lambda x)} +/
double integrate_exp(double lambda, double a, double b) {
  if (b == double.infinity) {
    return exp(-lambda*a);
  }
  return exp(-lambda*a) - exp(-lambda*b);
}

/++ Compute int_{a}{b}{lambda/2*exp(lambda*x)/(exp(lambda*m)-1)} +/
double integrate_h(double m, double lambda, double a, double b) {
  return (exp(lambda*a)-exp(lambda*b)) / (2-2*exp(lambda*m));
}

/++ Return the sign of x +/
int sgn(T)(T val)
if (isIntegral!T) {
  return (T(0)<val) - (val<T(0));
}

/++ Map in a way that even numbers in [-m, m] come first, from inside out, then odd numbers come from outside in +/
int map(int m, int x) {
  assert(m > 0);
  int n = ((m+1)/2) * 2 - 1; // largest odd number <= m
  if (is_odd(x)) {
    int nevens = (m/2) * 2 + 1; // number of even values in [-m, m]
    return nevens + (n-abs(x)) + (sgn(x)+1)/2;
  }
  return abs(x) + (sgn(x)-1)/2;
}

/++ Compute the number of bits needed to encode s using exponential golomb with parameter k +/
double exp_golomb_bits(int s, int k) {
  assert(s>=0 && k>=0);
  int d = cast(int)log2(1+s/(1<<k));
  return (d+1) + (d+k);
}

double golomb_bits(int s, int m) {
  assert(s>=0 && m>0);
  int d = s / m;
  return d+1 + log2(m);
}

/++ Estimate the exponential parameter and compare the code length on the last level, between:
  - uniform distribution, conditioned on the parent
  - exponential distribution, conditioned on the parent
  - exponential distribution across the entire level +/
void test_6(const string[] argv) {
  import std.array : array;
  writeln("Test 6");
  auto fq = read_and_process_array(argv);
  double lambda = 10000;
  //auto fq = generate_array_exponential(lambda);
  lambda = ml_exponential(fq);
  writeln("maximum likelihood exponential = ", lambda);
  /* collect statistic */
  int block_size = 256;
  int nsamples = cast(int)fq.length;
  int nblocks = (nsamples+block_size-1) / block_size;
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>max(a,b))(block_size, nblocks, fq); // NOTE: use the max operator
  double code_length1 = 0; // encode the left child using uniform distribution in [0, M]
  double code_length2 = 0; // encode the left-right difference using exponential distribution conditioned on M
  double code_length3 = 0; // encode the left child using "global" exponential distribution
  double code_length4 = 0; // exponential golomb
  double code_length5 = 0; // golomb
  double code_length6 = 0;
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    int l = tree.nlevels - 1;
    auto be = tree.index_range(l);
    for (int i = be[0]; i < be[1]; i += 2) {
      auto M = tree[(i-1)/2];
      if (M > 0) {
        code_length1 += log2(M+1) + 1;
        auto n = tree[i] - tree[i+1];
        if (n == 0) {
          code_length2 +=  log2(1/(2*integrate_h(M+0.5, lambda, 0, 0.5)));
        }
        else if (0<n && n <M) {
          code_length2 +=  log2(1/integrate_h(M+0.5, lambda, n-0.5, n+0.5));
        }
        else if (-M<n && n<0) {
          code_length2 +=  log2(1/integrate_h(M+0.5, lambda, -n-0.5, -n+0.5));
        }
        else if (n == -M) {
          code_length2 +=  log2(1/integrate_h(M+0.5, lambda, -n-0.5, -n+0.5));
        }
        else if (n == M) {
          code_length2 +=  log2(1/integrate_h(M+0.5, lambda, n-0.5, n+0.5));
        }

        code_length3 += log2(1/integrate_exp(lambda, tree[i], tree[i]+1));
        code_length4 += exp_golomb_bits(map(M, n), 2);
        code_length5 += golomb_bits(map(M, n), 6);
        int nn = map(M, n);
        if (nn == 0) {
          code_length6 += 1;
        }
        else {
          code_length6 += exp_golomb_bits(nn-1, 2) + 1;
        }
      }
    }
  }
  writeln("code length 1 = ", code_length1);
  writeln("code length 2 = ", code_length2);
  writeln("code length 3 = ", code_length3);
  writeln("code length 4 = ", code_length4);
  writeln("code length 5 = ", code_length5);
  writeln("code length 6 = ", code_length6);
}

// TODO: also estimate the exponential parameter and replot table 8
int main(const string[] argv) {
  try {
    //test_1(argv);
    //test_2(argv);
    //test_3();
    test_4(argv);
    //test_5(argv);
    //test_6(argv);
  }
  catch (Exception e) {
    writeln(e);
    return 1;
  }
  return 0;
}
