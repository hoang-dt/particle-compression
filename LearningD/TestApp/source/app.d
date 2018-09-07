import core.bitop;
import std.algorithm;
import std.algorithm.sorting;
import std.array;
import std.container.array;
import std.conv;
import std.datetime.stopwatch;
import std.exception;
import std.file;
import std.format;
import std.math;
import std.outbuffer;
import std.path;
import std.random;
import std.range;
import std.stdio;
import std.traits;
import std.typecons;
import dstats;
import arithmetic_coder;
import array;
import array_util;
import binary_tree;
import circular_queue;
import expected;
import io;
import kdtree;
import kdtree_haar;
import lorenzo;
import math;
import number;
import stats;
import bit_stream;


Array3D!double read_and_process_array2(const string[] argv, bool unsigned, bool shuffle) {
  import std.array : array;
  import std.stdio;
  enforce(argv.length == 2, "Args: [json metadata file]");
  string json_file = argv[1];
  /* read json data set */
  auto dataset = read_json(json_file);
  enforce(dataset, dataset.exception().toString());
  string dtype = dataset.metadata["dtype"].str;
  enforce(dtype=="float64", dtype ~ " not supported");
  auto f = dataset.data.get!(Array3D!double);
  lorenzo_predict(f);

  for (int i = 0; i < 10; ++i) {
    //cdf53_lift(f, i);
  }
  double[] fp;
  for (size_t i = 0; i < f.length; ++i) {
    if (f[i] >= 0) {
      fp ~= f[i];
    }
  }
  return new Array3D!double(f.dims, fp);
}
/++ Read a raw data set and process it into 16-bit unsigned, shuffled array of residuals +/
Array3D!int read_and_process_array(const string[] argv) {
  import std.array : array;
  import std.stdio;
  enforce(argv.length == 2, "Args: [json metadata file]");
  string json_file = argv[1];
  /* read json data set */
  auto dataset = read_json(json_file);
  enforce(dataset, dataset.exception().toString());
  string dtype = dataset.metadata["dtype"].str;
  enforce(dtype=="float64", dtype ~ " not supported");
  /* quantize to 15 bit integers */
  int bits = 20;
  auto f = dataset.data.get!(Array3D!double);
  //auto ff = filter!(a=>a>0)(f.buf_).array;
  //auto ab = estimate_gamma_parameters(ff);
  //writeln(ab[0], " hehe ", ab[1]);
  //write_text("residuals.txt", ff);
  auto fq = new Array3D!int(f.dims);
  //quantize_midtread(f, bits, fq);
  auto emax = quantize(f, bits, fq);
  /* generate 16-bit residuals using the Lorenzo predictor */
  lorenzo_predict(fq);

  //take_difference(fq);
  /* wavelet transform */
  //for (int i = 0; i < 10; ++i) {
    //cdf53_lift(fq, i);
  //}

  //{ // randomly flip the sign bit
  //  auto rnd = Random(unpredictableSeed);
  //  foreach (ref e; fq) {
  //    auto a = uniform(0, 2, rnd);
  //    e *= (2*a-1);
  //  }
  //}

  /* turn signed residuals into unsigned ones */
  foreach (ref int e; fq) {
    e = sign_to_lsb(e);
  }
  /* shuffle */
  //auto rnd = MinstdRand0(42);
  //fq.buf_ = fq.buf_.randomShuffle(rnd);
  //write_text("residuals.txt", fq);
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
  double lambda = estimate_exponential_parameter(fq);
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
  double lambda = estimate_exponential_parameter(f);
  writeln("lambda = ", lambda);
  return f;
}

/++ Generate n random particles in the [0, 1] box +/
ParticleArray!T generate_random_particles(T)(int n) {
  auto rnd = MinstdRand0(42);
  ParticleArray!T particles;
  particles.position.length = 1;
  for (int i = 0; i < n; ++i) {
    double x = uniform01(rnd);
    double y = uniform01(rnd);
    double z = uniform01(rnd);
    particles.position[0] ~= Vec3!T(x,y,z);
  }
  return particles;
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
void test_3(const string[] argv) {
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
  //auto lambda = ml_exponential_even(fq);
  auto lambda = estimate_exponential_parameter(fq);
  writeln("lambda = ", lambda);
  write_text("residuals.txt", fq);
  //auto fq = generate_array_exponential(1);
  /* collect statistic */
  int block_size = 256;
  int nsamples = cast(int)fq.length;
  int nblocks = (nsamples+block_size-1) / block_size;
  //auto trees = build_binary_trees!(typeof(fq), (a,b)=>a+b)(block_size, nblocks, fq);
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>max(a,b))(block_size, nblocks, fq);
  alias map = int[double];
  auto counts = new map[](trees[0].nlevels); // one map per level
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 0; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      for (int i = be[0]; i < be[1]; ++i) {
        auto v = tree[i];
        ++counts[l][v];
      }
    }
  }

  /* compute one mode for each level */
  int[] modes = new int[](counts.length);
  for (int l = 0; l+1 < modes.length; ++l) {
    auto r = counts[l].byValue.array;
    topN!"a>b"(r, 7);
    modes[l] = r[7];
    writeln("mode ", l, " ", modes[l]);
  }
  auto output = new double[][](modes.length); // one array per level, of samples whose parent is equal to the mode of the previous level
  double[] m = new double[](modes.length);
  for (int l = 1; l < m.length; ++l) {
    m[l] = counts[l-1].byKey.filter!(k=>counts[l-1][k]==modes[l-1]).array[0];
    writeln("m ", l, " " ,m[l]);
  }
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    for (int l = 1; l < tree.nlevels; ++l) {
      auto be = tree.index_range(l);
      for (int i = be[0]; i < be[1]; i += 2) {
        auto p = tree[(i-1)/2];
        //enforce(p == (tree[i] + tree[i+1]));
        if (p == m[l]) {
          output[l] ~= tree[i] - tree[i+1]; // NOTE: this is used to plot (left child - right child)
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
  double lambda = estimate_exponential_parameter(fq);
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
  enforce(lambda*a < 700);
  enforce(lambda*b < 700);
  if (b == double.infinity) {
    return exp(-lambda*a);
  }
  return exp(-lambda*a) - exp(-lambda*b);
}

/++ Compute int_{a}{b}{lambda/2*exp(lambda*x)/(exp(lambda*m)-1)} +/
double integrate_h(double m, double lambda, double a, double b) {
  enforce(lambda*a < 700);
  enforce(lambda*m < 700);
  enforce(lambda*b < 700);
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
// TODO: use a different method to approximate lambda (maximum spacing estimator or exponential regression)
// TODO: keep the laplace distribution, encode left-right conditioned on max(|left|, |right|)
void test_6(const string[] argv) {
  import std.array : array;
  writeln("Test 6");
  auto fq = read_and_process_array(argv);
  double lambda = 10000;
  //auto fq = generate_array_exponential(lambda);
  lambda = ml_exponential_even(fq);
  //lambda *= 2;
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
  int[] diffs;
  int[] maxes;
  double[int] code_length_per_max; // accumulate the code length per max
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    int l = tree.nlevels - 1;
    auto be = tree.index_range(l);
    for (int i = be[0]; i < be[1]; i += 2) {
      auto M = tree[(i-1)/2];
      if (M > 0) {
        double incr1 = log2(2*M+1);
        code_length1 += incr1;
        auto n = tree[i] - tree[i+1];
        double incr2 = 0;
        if (n == 0) {
          incr2 =  log2(1/(2*integrate_h(M+0.5, lambda, 0, 0.5)));
        }
        else if (0<n && n <=M) {
          incr2 =  log2(1/integrate_h(M+0.5, lambda, n-0.5, n+0.5));
        }
        else if (-M<=n && n<0) {
          incr2 =  log2(1/integrate_h(M+0.5, lambda, -n-0.5, -n+0.5));
        }
        enforce(incr2 > 0);
        code_length2 += incr2;
        code_length_per_max[M] += incr2;
        code_length3 += log2(1/integrate_exp(lambda, tree[i], tree[i]+1));
        code_length4 += exp_golomb_bits(map(M, n), 2);
        code_length5 += golomb_bits(map(M, n), 6);
        int nn = map(M, n);
        code_length6 += min(incr1, incr2);
        diffs ~= tree[i] - tree[i+1];
        maxes ~= M;
      }
    }
  }
  double code_length7 = 0;
  auto probs = compute_probabilities(diffs);
  foreach (e; probs) {
    code_length7 += e*log2(1/e);
  }
  code_length7 *= diffs.length;
  probs = compute_probabilities(maxes);
  double subtract = 0;
  foreach (e; probs) {
    subtract += e*log2(1/e);
  }
  write_text("code_length_per_max.txt", code_length_per_max);
  //writeln("num maxes = ", maxes.length);
  subtract *= maxes.length;
  code_length7 -= subtract;
  writeln("code length 1 = ", code_length1);
  writeln("code length 2 = ", code_length2);
  writeln("code length 3 = ", code_length3);
  writeln("code length 4 = ", code_length4);
  writeln("code length 5 = ", code_length5);
  writeln("code length 6 = ", code_length6);
  writeln("code length 7 = ", code_length7, " ", subtract);
}

/++ Given the max, we encode the difference between D(left, max) and D(right, max) where D is the
function that computes how many bits its arguments agree (from the msb) +/
void test_7(const string[] argv) {
  import std.array : array;
  writeln("Test 7");
  auto fq = read_and_process_array(argv);
  int block_size = 256;
  int nsamples = cast(int)fq.length;
  int nblocks = (nsamples+block_size-1) / block_size;
  int[int] output; // map from bit plane differences to count
  auto trees = build_binary_trees!(typeof(fq), (a,b)=>max(a,b))(block_size, nblocks, fq); // NOTE: use the max operator
  for (int b = 0; b < nblocks; ++b) {
    auto tree = trees[b];
    int l = tree.nlevels - 1;
    auto be = tree.index_range(l);
    for (int i = be[0]; i < be[1]; i += 2) {
      auto left = tree[i];
      auto right = tree[i+1];
      auto m = tree[(i-1)/2];
      auto xleft = bsr(left^m) + 1;
      auto xright = bsr(right^m) + 1;
      auto xor = bsr(left^right) + 1; // number of different bits from the right
      auto bb = bsr(m)+1;
      if (m == 0) {
        continue;
      }
      if (left==m && right==m) {
        output[bb] += 1;
      }
      else {
        output[bb-xor] += 1;
      }
    }
  }
  int sum = reduce!((a,b) => a + b)(0, output.byValue());
  writeln(sum);
  //double code_length1 = 0;
  //double code_length2 = 0;
  //for (int b = 0; b < nblocks; ++b) {
  //  auto tree = trees[b];
  //  int l = tree.nlevels - 1;
  //  auto be = tree.index_range(l);
  //  for (int i = be[0]; i < be[1]; i += 2) {
  //    auto left = tree[i];
  //    auto right = tree[i+1];
  //    auto m = tree[(i-1)/2];
  //    auto bb = bsr(m)+1+1;
  //    if (m == 0) {
  //      continue;
  //    }
  //    auto xor = bsr(left^right) + 1; // number of different bits from the right
  //    auto bbb = bsr(m)+1;
  //    if (left==m && right==m) {
  //      code_length2 += log2(double(sum)/(output[bbb]));
  //    }
  //    else if (left == m) {
  //      code_length2 += log2(double(sum)/(output[bbb-xor]))+1+xor;
  //    }
  //    else if (right == m) {
  //      code_length2 += log2(double(sum)/(output[bbb-xor]))+1+xor;
  //    }
  //    //code_length2 += log2(37);
  //    code_length1 += log2(2*m+1);
  //  }
  //}
  //writeln(code_length1);
  //writeln(code_length2);
  //for
  File file = File("test_7.txt", "w");
  foreach (e; output.byKeyValue().array.sort!"a.key<b.key") {
    file.writeln(e.key, " ", e.value);
  }
}

/++ Estimate the gamma parameters and exponential parameter +/
void test_8(const string[] argv) {
  import std.array : array;
  writeln("Test 8");
  auto fq = read_and_process_array(argv);
  auto fqf = filter!(a =>a>0)(fq).array;
  auto ab = estimate_gamma_parameters(fqf);
  auto lambda = estimate_exponential_parameter(fqf);
  writeln("Gamma parameters ", ab[0], " ", ab[1]);
  writeln("Exponential parameter ", lambda);
  write_text("residuals.txt", fqf);
}

ParticleArray!float load_particles(const string[] argv) {
  ParticleArray!float particles;
  auto ext = extension(argv[2]);
  if (ext == ".gro") {
    particles = parse_gro!float(argv[2]);
  }
  else if (ext == ".xyz") {
    particles = parse_xyz!float(argv[2]);
  }
  else if (ext == ".vtu") {
    particles = parse_vtu(argv[2]);
  }
  else if (ext == ".dat") {
    particles = parse_cosmo(argv[2]);
  }
  else if (ext == ".hex") {
    auto p = parse_hex_meshes(argv[2]);
    particles = convert_type!float(p);
  }
  else {
    enforce(argv.length==4, "need number of particles to generate");
    particles = generate_random_particles!float(to!int(argv[3]));
  }
  writeln("done parsing");
  return particles;
}

/++ Collect statistics on the kd-tree +/
void test_9(const string[] argv) {
  import std.array : array;
  writeln("Test 9");
  enforce(argv.length>=3, "usage: "~argv[0]~" [test_9] [particle file]");

  void traverse_code_length(T, int R)(int level, KdTree!(T,R) parent, ref Tuple!(double[], double) code_length1, ref Tuple!(double[], double) code_length2) {
    int N = parent.end_ - parent.begin_;
    if (N == 1) { // leaf level
      code_length1[1] += parent.bits_.length;
      code_length2[1] += parent.bits_.length;
    }
    else { // non leaf
      if (code_length1[0].length <= level) {
        ++code_length1[0].length;
        code_length1[0][level] = 0;
      }
      if (code_length2[0].length <= level) {
        ++code_length2[0].length;
        code_length2[0][level] = 0;
      }
      int n = parent.left_ is null ? 0 : parent.left_.end_ - parent.left_.begin_;
      code_length1[0][level] += N - log2_C_n_m(N, n);
      if (parent.left_ !is null) {
        traverse_code_length(level+1, parent.left_, code_length1, code_length2);
      }
      if (parent.right_ !is null) {
        traverse_code_length(level+1, parent.right_, code_length1, code_length2);
      }
    }
  }

  /* read particle files, or generate random particles */
  auto particles = load_particles(argv);
  Tuple!(double[], double) code_length1; code_length1[1] = 0;
  Tuple!(double[], double) code_length2; code_length2[1] = 0;
  for (size_t i = 0; i < 1/*particles.position.length*/; ++i) {
    auto tree = new KdTree!float();
    tree.set_accuracy(0.2);
    //tree.set_precision(23);
    //tree.set_accuracy(1e-5);
    tree.build!"xyz"(particles.position[i]); // build a tree from the first time step
    traverse_code_length(0, tree, code_length1, code_length2);
    //int[][int] stats;
    //traverse(tree, stats);
    //auto results = sort!((a,b)=>(a.value.length>b.value.length))(stats.byKeyValue().array);
    //foreach (e; results) {
    //  if (e.key == 15) {
    //    foreach (v; e.value) {
    //      writeln(v);
    //    }
    //  }
    //}
  }
  writeln("code length 1 = ", code_length1);
  writeln("code length 2 = ", code_length2);
  for (size_t i = 0; i < code_length1[0].length; ++i) {
    writeln(code_length1[0][i] / code_length2[0][i]);
  }
  writeln(code_length1[1] / code_length2[1]);
  writeln(reduce!((a,b)=>a+b)(code_length1[0]) + code_length1[1]);
  writeln(reduce!((a,b)=>a+b)(code_length2[0]) + code_length2[1]);
}

/++ Test the kdtree to make sure it is working +/
void test_10(const string[] argv) {
  import std.array : array;
  writeln("Test 10");

  double particle_rmse(T)(const Vec3!T[] points1, const Vec3!T[] points2) {
    assert(points1.length==points2.length && points1.length>0);
    double rmse = 0;
    for (size_t i = 0; i < points1.length; ++i) {
      auto p = points1[i] - points2[i];
      rmse += sqr_norm(p);
    }
    rmse /= (points1.length * 3);
    return sqrt(rmse);
  }

  auto particles = load_particles(argv);
  writeln(particles.position[0].length);
  Vec3!float[] output_particles;
  for (size_t i = 0; i < particles.position.length; ++i) { // time step loop
    auto tree = new KdTree!float();
    //tree.set_accuracy(1e-3);
    tree.build!"xyz"(particles.position[i]); // build a tree from the first time step
    kdtree_to_particles(tree, output_particles);
    writeln(output_particles.length);
    writeln(particles.position[i].length);
    enforce(output_particles.length == particles.position[i].length);
    writeln("rmse = ", particle_rmse(particles.position[i], output_particles));
  }
}

/++ Collect and print tree statistics +/
void test_11(const string[] argv) {
  import std.array : array;
  writeln("Test 11");

  void traverse(T, int R)(KdTree!(T,R) parent, ref int[][int] stats) {
    int N = parent.end_ - parent.begin_;
    if (parent.left_ !is null) {
      int n = parent.left_.end_ - parent.left_.begin_; // number of particles in the parent
      stats[N] ~= n;
    }
    if (parent.left_ !is null) {
      traverse(parent.left_, stats);
    }
    if (parent.right_ !is null) {
      traverse(parent.right_, stats);
    }
  }

  auto particles = load_particles(argv);
  for (size_t i = 0; i < 1; ++i) { // time step loop
    auto tree = new KdTree!float();
    //tree.set_accuracy(1e-3);
    tree.build!"xyz"(particles.position[i]); // build a tree from the first time step
    int[][int] stats;
    traverse(tree, stats);
    auto results = sort!((a,b)=>(a.value.length>b.value.length))(stats.byKeyValue().array);
    foreach (e; take(results, 8).reverse) {
      writeln(e.key, "---------------------------");
      foreach (v; e.value) {
        writeln(v);
      }
      break;
    }
  }
}

/++ Construct one histogram per tree, of the ratio between left/parent +/
void test_12(const string[] argv) {
  import std.array : array;
  writeln("Test 12");

  void traverse(T, int R)(KdTree!(T,R) parent, ref double[8] histogram) {
    int N = parent.end_ - parent.begin_;
    if (N < 8) {
      return;
    }
    if (parent.left_ !is null) {
      int n = parent.left_.end_ - parent.left_.begin_; // number of particles in the parent
      int k = cast(int)(double(n) / double(N) * 7.0);
      ++histogram[k];
    }
    if (parent.left_ !is null) {
      traverse(parent.left_, histogram);
    }
    if (parent.right_ !is null) {
      traverse(parent.right_, histogram);
    }
  }

  auto particles = load_particles(argv);
  for (size_t i = 0; i < 1; ++i) { // time step loop
    auto tree = new KdTree!float();
    //tree.set_accuracy(1e-3);
    tree.build!"xyz"(particles.position[i]); // build a tree from the first time step
    double[8] histogram = 0;
    traverse(tree, histogram);
    foreach (e; histogram) {
      writeln(e);
    }
  }
}

/++ Write the x-component of the velocity/concentration field in depth-first order +/
void test_13(const string[] argv) {
  import std.array : array;
  writeln("Test 13");

  auto particles = load_particles(argv);
  if (extension(argv[2]) == ".gro") {
    enforce((particles.velocity !is null) && (particles.velocity.length == particles.position.length));
    for (size_t i = 0; i < 1; ++i) { // time step loop
      auto tree = new KdTree!float();
      tree.build!"xyz"(particles.position[i], particles.velocity[i]); // build a tree from the first time step
      foreach (e; particles.velocity[i]) {
        writeln(e.x);
      }
    }
  }
  else if (extension(argv[2]) == ".vtu") {
    enforce((particles.concentration !is null) && (particles.concentration.length == particles.concentration.length));
    for (size_t i = 0; i < 1; ++i) { // time step loop
      auto tree = new KdTree!float();
      foreach (e; particles.concentration[i]) {
        writeln(e);
      }
      tree.build!"xyz"(particles.position[i], particles.concentration[i]); // build a tree from the first time step
    }
  }
  else {
    enforce(false);
  }
}

/++ Rendering test +/
void test_14(const string[] argv) {
  writeln("Test 14");
  enforce(argv.length == 5);
  auto particles = load_particles(argv);
  auto tree = new KdTree!float();
  float accuracy = to!float(argv[4]);
  tree.set_accuracy(accuracy);
  tree.build!"xyz"(particles.position[0]);
  Vec3!float[] points;
  kdtree_to_particles(tree, points);
  ParticleArray!float new_particles;
  new_particles.position ~= points;
  dump_xyz(argv[3], new_particles.position[0]);
}


/++ Build two kdtree and output the differences between them +/
void test_15(const string[] argv) {
  writeln("Test 15");
  auto particles = parse_gro!float("D:/Datasets/particles/alfredo/t100-t101.gro");
  auto tree1 = new KdTree!float(); tree1.build!"xyz"(particles.position[0]);
  auto tree2 = new KdTree!float(); tree2.build_with_bbox!"xyz"(particles.position[1], tree1.bbox_);
  LeafChange[] changes;
  compare_kdtrees(tree1, tree2, changes);
  foreach (e; changes) {
    //writeln(cast(int)(e));
  }
  writeln("abc");
  int[] diffs;
  compare_kdtrees_values(tree1, tree2, diffs);
  foreach (e; diffs) {
    writeln(e);
  }
}
/++ Convert every format to xyz +/
void test_16(const string[] argv) {
  writeln("Test 16 (format conversion)");
  enforce(argv.length == 4);
  auto particles = load_particles(argv);
  dump_xyz(argv[3], particles.position[0]);
}

/++ Build the Haar-based KdTree +/
void test_17(const string[] argv) {
  writeln("Test 17 (Haar-based Kdtree)");
  auto particles = load_particles(argv);
  auto tree = new KdTreeHaar!float();
  tree.build!"xyz"(particles.position[0]);
  writeln(particles.position[0][0]);
  writeln(particles.position[0][1]);
  writeln("--------------");
  tree.pad();
  tree.haar_transform();
  Vec3!float[] out_particles;
  tree.to_particles(out_particles);
  writeln(out_particles[0]);
  writeln(out_particles[1]);
  writeln("--------------");
  out_particles.length = 0;
  Vec3!float[] out_particles2;
  //tree.zero_out_levels(1);
  Subband!float[int] items;
  tree.organize_subbands(items);
  writeln(items.length);
  std.file.remove("x-level.txt");
  std.file.remove("y-level.txt");
  std.file.remove("z-level.txt");
  for (int i = 1; i <= items.length; ++i) {
    auto item = items[i].dup;
    sort!((a,b)=>a[0]<b[0])(item);
    write_text("x-level.txt", std.algorithm.iteration.map!"a[1].x"(item), "a");
    write_text("y-level.txt", std.algorithm.iteration.map!"a[1].y"(item), "a");
    write_text("z-level.txt", std.algorithm.iteration.map!"a[1].z"(item), "a");
  }

  Vec3!float[][14] items2;
  for (int i = 1; i <= items.length; ++i) {
    auto item = items[i].dup;
    for (int j = 0; j < item.length; ++j) {
      enforce(item[j][0] != SubbandType.INVALID);
      items2[item[j][0]] ~= item[j][1];
    }
  }
  for (int i = 0; i < 14; ++i) {
    write_text("x-subband"~to!string(i)~".txt", std.algorithm.iteration.map!"a.x"(items2[i]));
    write_text("y-subband"~to!string(i)~".txt", std.algorithm.iteration.map!"a.y"(items2[i]));
    write_text("z-subband"~to!string(i)~".txt", std.algorithm.iteration.map!"a.z"(items2[i]));
  }


  //tree.invert_haar_transform();
  //tree.to_particles(out_particles2);
  //writeln(out_particles2[0]);
  //writeln(out_particles2[1]);
  //writeln(out_particles.length);
  //dump_xyz(argv[3], out_particles2);

  Vec3!float[] vals;
  tree.list_vals(vals);
  write_text("vals_x.txt", std.algorithm.iteration.map!"a.x"(vals));
  write_text("vals_y.txt", std.algorithm.iteration.map!"a.y"(vals));
  write_text("vals_z.txt", std.algorithm.iteration.map!"a.z"(vals));
}

/++ Assuming particles are grouped every k time steps, create a surrogate particle data set, by
averaging the positions of the particles in n consecutive time steps.
Return: a tuple of the surrogate particles and the maximum distance among k same particles +/
Tuple!(ParticleArray!T, Vec3!T) create_surrogate_particles(T)(ParticleArray!T particles, int n) {
  ParticleArray!T out_particles;
  Vec3!T max_dist = Vec3!T(0, 0, 0);
  out_particles.position.length = (particles.position.length + n-1) / n;
  for (int i = 0; i < out_particles.position.length; ++i) { // time step loop
    out_particles.position[i].length = particles.position[i].length;
    out_particles.position[i][] = Vec3!T(0, 0, 0);
    int m = min(particles.position.length-i*n, n);
    for (int k = 0; k < m; ++k) { // inner time step loop
      for (int j = 0; j < out_particles.position[i].length; ++j) { // particle loop
        out_particles.position[i][j] = out_particles.position[i][j] + particles.position[i*n+k][j];
      }
    }
    for (int j = 0; j < out_particles.position[i].length; ++j) { // particle loop
      out_particles.position[i][j] = out_particles.position[i][j] / T(m);
    }
    /* update the maximum distance */
    if (i == 0) {// TODO: swap the two for statements
      for (int j = 0; j < out_particles.position[i].length; ++j) {
        Vec3!T local_max = Vec3!T(0, 0, 0);
        for (int k = 0; k < m; ++k) {
        //int j = 0; {
          auto p = particles.position[i*n+k][j];
          auto mid = out_particles.position[i][j];
          max_dist.x = max(max_dist.x, abs(p.x-mid.x));
          max_dist.y = max(max_dist.y, abs(p.y-mid.y));
          max_dist.z = max(max_dist.z, abs(p.z-mid.z));
          local_max.x = max(local_max.x, abs(p.x-mid.x));
          local_max.y = max(local_max.y, abs(p.y-mid.y));
          local_max.z = max(local_max.z, abs(p.z-mid.z));
        }
        writeln(j, " ", local_max.x);
      }
    }
  }
  return tuple(out_particles, max_dist);
}

/++ Compare the distance among particles spatially and temporally +/
void test_18(const string[] argv) {
  writeln("Test 18 (grouping of particles)");
  enforce(argv.length == 4, "Usage: [exe] [test] [data file] [group size]");
  auto particles = load_particles(argv);
  int group_size = to!int(argv[3]);
  enforce(particles.position.length > 1, "not enough particles");
  auto tree = new KdTree!float();
  auto result = create_surrogate_particles(particles, group_size);
  auto combined_particles = result[0];
  auto max_dist = result[1];
  tree.build!"xyz"(combined_particles.position[0]);
  Vec3!float[] bbox_sizes;
  kdtree_to_bbox_sizes(tree, bbox_sizes);
  auto avg_size = avg_bbox_size(bbox_sizes);
  writeln("group size = ", group_size);
  writeln("max dist = ", max_dist);
  writeln("avg size = ", avg_size);
}

/++ Output x trajectory +/
void test_19(const string[] argv) {
  writeln("Test 19 (x trajectory)");
  enforce(argv.length == 3);
  auto particles = parse_gro!float(argv[2]);
  undo_periodic_boundary(particles);
  int pid = 8000; // particle id
  float[] x_trajectory, y_trajectory, z_trajectory;
  for (int i = 0; i < particles.position.length; ++i) {
    x_trajectory ~= particles.position[i][pid].x;
    y_trajectory ~= particles.position[i][pid].y;
    z_trajectory ~= particles.position[i][pid].z;
  }
  write_text("x-trajectories.txt", x_trajectory);
  write_text("y-trajectories.txt", y_trajectory);
  write_text("z-trajectories.txt", z_trajectory);
  float[] x_velocity, y_velocity, z_velocity;

  for (int i = 0; i < particles.velocity.length; ++i) {
    x_velocity ~= particles.velocity[i][pid].x;
    y_velocity ~= particles.velocity[i][pid].y;
    z_velocity ~= particles.velocity[i][pid].z;
  }
  write_text("x-velocities.txt", x_velocity);
  write_text("y-velocities.txt", y_velocity);
  write_text("z-velocities.txt", z_velocity);

  float[] x_velocity_predict = new float[](x_velocity.length);
  float[] y_velocity_predict = new float[](y_velocity.length);
  float[] z_velocity_predict = new float[](z_velocity.length);
  float dt = 0.001;
  for (int i = 0; i+1 < x_velocity.length; ++i) {
    x_velocity_predict[i] = 2 * (x_trajectory[i+1]-x_trajectory[i])/dt - x_velocity[i];
    y_velocity_predict[i] = 2 * (y_trajectory[i+1]-y_trajectory[i])/dt - y_velocity[i];
    z_velocity_predict[i] = 2 * (z_trajectory[i+1]-z_trajectory[i])/dt - z_velocity[i];
  }
  /* normalize the predicted velocities */
  for (int i = 0; i+1 < x_velocity.length; ++i) {
    float x = x_velocity_predict[i], y = y_velocity_predict[i], z = z_velocity_predict[i];
    //float length = sqrt(x*x + y*y + z*z);
    //if (length > 0) {
    //  x_velocity_predict[i] /= length;
    //  y_velocity_predict[i] /= length;
    //  z_velocity_predict[i] /= length;
    //}
  }
  write_text("x-velocities-predicted.txt", x_velocity_predict);
  write_text("y-velocities-predicted.txt", y_velocity_predict);
  write_text("z-velocities-predicted.txt", z_velocity_predict);
  /* normalize the original velocities */
  for (int i = 0; i < x_velocity.length; ++i) {
    float x = x_velocity[i], y = y_velocity[i], z = z_velocity[i];
    //float length = sqrt(x*x + y*y + z*z);
    //if (length > 0) {
    //  x_velocity[i] /= length;
    //  y_velocity[i] /= length;
    //  z_velocity[i] /= length;
    //}
  }
  write_text("x-velocities-normalized.txt", x_velocity);
  write_text("y-velocities-normalized.txt", y_velocity);
  write_text("z-velocities-normalized.txt", z_velocity);
}

/++  +/
void test_20(const string[] argv) {
  writeln("Test 20");
  auto particles = load_particles(argv);
  auto tree1 = new KdTree!float();
  tree1.build!"xyz"(particles.position[0]);
  Vec3!float[] particles1;
  kdtree_to_particles(tree1, particles1);
  auto tree2 = new KdTree!float();
  tree2.set_accuracy(1e-4);
  tree2.build!"xyz"(particles.position[0]);
  Vec3!float[] particles2;
  kdtree_to_particles(tree2, particles2);
  auto particles3 = new Vec3!float[](particles1.length);
  for (size_t i = 0; i < particles1.length; ++i) {
    particles3[i] = particles1[i] - particles2[i];
    auto p = particles3[i];
    particles3[i].x = sqrt(p.x*p.x+p.y*p.y+p.z*p.z);
  }
  write_text("diff_x.txt", std.algorithm.iteration.map!"a.x"(particles3));
  write_text("diff_y.txt", std.algorithm.iteration.map!"a.y"(particles3));
  write_text("diff_z.txt", std.algorithm.iteration.map!"a.z"(particles3));
}

void test_random() {
  auto rnd = Random(unpredictableSeed);

  // Generate a float in [0, 1)
  float[] v = new float[](100000);
  for (int i = 0; i < 100000; ++i) {
    auto a = uniform(0.0f, 1.0f, rnd);
    auto b = uniform(0.0f, 1.0f, rnd);
    v[i] = (a-0.5)*(a-0.5) + (b-0.5)*(b-0.5);
  }
  write_text("v.txt", v);
}

// TODO: also estimate the exponential parameter and replot table 8
// TODO: plot similar plots using actual exponential distributions
// TODO: where to refine next? (plot the psnr curve)
// TODO: serialize the tree
// TODO: encode tree differences across time steps
// TODO: 6D particle?
// TODO: use velocity as input into kdtree
// TODO: compression over time, maybe using 4-dimension kdtree?

struct Test(int n) { static const int N = n; int b; }


char[] my_filter(string s) {
  char[] output;
  foreach (c; s) {
    if (0<=c && c<=127) {
      output ~= c;
    }
  }
  return output;
}

void test_21(const string[] argv) {
  import particle_compression;
  writeln("Test 21");
  auto particles = load_particles(argv);
  BitStream bs;
  encode(particles.position[0], bs);
  writeln(bs.size());
}

import math;

// TODO: 1 gives infinity
int main(const string[] argv) {
  //const int N = 2;
  //float m = float(N) / 2; // mean
  //float s = sqrt(float(N)) / 2; // standard deviation
  //const float bin_width = 0.5f * (math.erfc(-0.5f/s/sqrt(2.0f)) - math.erfc(0.5f/s/sqrt(2.0f)));
  //int nb = cast(int)ceil(1.0f / bin_width);
  //const int nbits = 6;
  //const int nbins = 1<<nbits;
  //float[nbins] lookup_table;
  //float step = 1.0f / nbins;
  //for (int i = 0; i < nbins; ++i) {
  //  float y = (i+1) * step;
  //  lookup_table[i] = erfinv(2*y-1) * sqrt(2.0f);
  //}
  //auto file = File("table.txt", "w");
  //for (int i = 0; i < nbins; ++i) {
  //  float y = (i+1) * step;
  //  file.writeln(y, " ", lookup_table[i]);
  //}
  //return 0;
  //string raw_contents = readText("E:/Workspace/binomial-coding/test.txt");
  //char[] contents = my_filter(raw_contents);
  //alias Model = CharModel!(uint, 17, 15);
  //Model m;
  //m.collect_probs(contents);
  //ArithmeticCoder!Model coder;
  //coder.set_model(m);
  //coder.encode(contents);
  //char[] output;
  //coder.decode(output);
  //writeln(output);
  //Test!1 test;
  //writeln(test.N);
  //ulong n1 = ~(ulong(true)-1);
  //ulong n2 = ~(ulong(false)-1);
  //writefln("%64b", n1);
  //writefln("%64b", 14);
  //BitStream bs;
  //bs.init_write(100);
  //bs.repeated_write(true, 172);
  //bs.flush();
  //bs.rewind();
  //bs.init_read();
  //ulong n = 0;
  //n = bs.read(57);
  //writefln("%64b", n);
  //n = bs.read(57);
  //writefln("%64b", n);
  //n = bs.read(57);
  //writefln("%64b", n);
  //n = bs.read(2);
  //writefln("%64b", n);
  //return 0;
  //float[] a = new float[](3);
  //a[] = 0;
  alias test_func = void function(const string[]);
  //writeln(a);
  test_func[string] func_map;
  func_map["test_1"] = &test_1;
  func_map["test_2"] = &test_2;
  func_map["test_3"] = &test_3;
  func_map["test_4"] = &test_4;
  func_map["test_5"] = &test_5;
  func_map["test_6"] = &test_6;
  func_map["test_7"] = &test_7;
  func_map["test_8"] = &test_8;
  func_map["test_9"] = &test_9;
  func_map["test_10"] = &test_10;
  func_map["test_11"] = &test_11;
  func_map["test_12"] = &test_12;
  func_map["test_13"] = &test_13;
  func_map["test_14"] = &test_14;
  func_map["test_15"] = &test_15;
  func_map["test_16"] = &test_16;
  func_map["test_17"] = &test_17;
  func_map["test_18"] = &test_18;
  func_map["test_19"] = &test_19;
  func_map["test_20"] = &test_20;
  func_map["test_21"] = &test_21;
  //BitStream bs;
  //bs.init_write(100);
  //bs.write(0xABCDEF, 25);
  //bs.write(0xABABABAB, 33);
  //bs.write(0xABCDEF, 25);
  //bs.write(0xFFF, 13);
  //bs.write(0xEE, 0);
  //bs.write(0xAAAAAAAAAAAAAA, 57);
  //bs.write(0xB, 5);
  //bs.flush();
  ////bs.rewind();
  //bs.init_read();
  //writefln("%x", bs.peek(25));
  //bs.consume(25);
  //bs.refill();
  //writefln("%x", bs.peek(33));
  //bs.consume(33);
  //bs.refill();
  //writefln("%x", bs.peek(25));
  //bs.consume(25);
  //bs.refill();
  //writefln("%x", bs.peek(13));
  //bs.consume(13);
  //bs.refill();
  //writefln("%x", bs.peek(57));
  //bs.consume(57);
  //bs.refill();
  //writefln("%x", bs.peek(5));
  //bs.consume(5);
  try {
    func_map[argv[1]](argv);
    int stop = 0;
  }
  catch (Exception e) {
    writeln(e);
    return 1;
  }
  return 0;
}
