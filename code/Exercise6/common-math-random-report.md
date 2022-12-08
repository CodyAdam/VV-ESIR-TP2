# Report CC Value

Calculating the CC values of the classes in `/home/cody/Git/VV-ESIR-TP2/code/Exercise2/source/commons-math3-3.6.1-src/src/main/java/org/apache/commons/math3/random`
# [Histogram of CC values per class](#histogram-of-cc-values-per-class)

# The class 'RandomGenerator'
Package: `org.apache.commons.math3.random.RandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[int[] seed]` | $1$ |
| setSeed | `[long seed]` | $1$ |
| nextBytes | `[byte[] bytes]` | $1$ |
| nextInt | `[]` | $1$ |
| nextInt | `[int n]` | $1$ |
| nextLong | `[]` | $1$ |
| nextBoolean | `[]` | $1$ |
| nextFloat | `[]` | $1$ |
| nextDouble | `[]` | $1$ |
| nextGaussian | `[]` | $1$ |
| **Total CC** || **$11$** |



# The class 'AbstractRandomGenerator'
Package: `org.apache.commons.math3.random.AbstractRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| clear | `[]` | $1$ |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[int[] seed]` | $2$ |
| setSeed | `[long seed]` | $1$ |
| nextBytes | `[byte[] bytes]` | $5$ |
| nextInt | `[]` | $1$ |
| nextInt | `[int n]` | $2$ |
| nextLong | `[]` | $1$ |
| nextBoolean | `[]` | $1$ |
| nextFloat | `[]` | $1$ |
| nextDouble | `[]` | $1$ |
| nextGaussian | `[]` | $4$ |
| **Total CC** || **$21$** |



# The class 'Well19937c'
Package: `org.apache.commons.math3.random.Well19937c`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'RandomDataGenerator'
Package: `org.apache.commons.math3.random.RandomDataGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextHexString | `[int len]` | $4$ |
| nextInt | `[final int lower, final int upper]` | $1$ |
| nextLong | `[final long lower, final long upper]` | $10$ |
| nextLong | `[final RandomGenerator rng, final long n]` | $4$ |
| nextSecureHexString | `[int len]` | $5$ |
| nextSecureInt | `[final int lower, final int upper]` | $1$ |
| nextSecureLong | `[final long lower, final long upper]` | $10$ |
| nextPoisson | `[double mean]` | $1$ |
| nextGaussian | `[double mu, double sigma]` | $2$ |
| nextExponential | `[double mean]` | $1$ |
| nextGamma | `[double shape, double scale]` | $1$ |
| nextHypergeometric | `[int populationSize, int numberOfSuccesses, int sampleSize]` | $1$ |
| nextPascal | `[int r, double p]` | $1$ |
| nextT | `[double df]` | $1$ |
| nextWeibull | `[double shape, double scale]` | $1$ |
| nextZipf | `[int numberOfElements, double exponent]` | $1$ |
| nextBeta | `[double alpha, double beta]` | $1$ |
| nextBinomial | `[int numberOfTrials, double probabilityOfSuccess]` | $1$ |
| nextCauchy | `[double median, double scale]` | $1$ |
| nextChiSquare | `[double df]` | $1$ |
| nextF | `[double numeratorDf, double denominatorDf]` | $1$ |
| nextUniform | `[double lower, double upper]` | $1$ |
| nextUniform | `[double lower, double upper, boolean lowerInclusive]` | $6$ |
| nextPermutation | `[int n, int k]` | $3$ |
| nextSample | `[Collection<?> c, int k]` | $4$ |
| reSeed | `[long seed]` | $1$ |
| reSeedSecure | `[]` | $1$ |
| reSeedSecure | `[long seed]` | $1$ |
| reSeed | `[]` | $1$ |
| setSecureAlgorithm | `[String algorithm, String provider]` | $1$ |
| getRandomGenerator | `[]` | $2$ |
| initRan | `[]` | $1$ |
| getSecRan | `[]` | $2$ |
| **Total CC** || **$74$** |



# The class 'Well19937a'
Package: `org.apache.commons.math3.random.Well19937a`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'AbstractWell'
Package: `org.apache.commons.math3.random.AbstractWell`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[final int seed]` | $1$ |
| setSeed | `[final int[] seed]` | $4$ |
| setSeed | `[final long seed]` | $1$ |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$7$** |



# The class 'SobolSequenceGenerator'
Package: `org.apache.commons.math3.random.SobolSequenceGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| initFromStream | `[final InputStream is]` | $6$ |
| initDirectionVector | `[final int d, final int a, final int[] m]` | $4$ |
| nextVector | `[]` | $4$ |
| skipTo | `[final int index]` | $8$ |
| getNextIndex | `[]` | $1$ |
| **Total CC** || **$23$** |



# The class 'RandomVectorGenerator'
Package: `org.apache.commons.math3.random.RandomVectorGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextVector | `[]` | $1$ |
| **Total CC** || **$1$** |



# The class 'EmpiricalDistribution'
Package: `org.apache.commons.math3.random.EmpiricalDistribution`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| load | `[double[] in]` | $1$ |
| load | `[URL url]` | $2$ |
| load | `[File file]` | $1$ |
| fillBinStats | `[final DataAdapter da]` | $4$ |
| findBin | `[double value]` | $1$ |
| getNextValue | `[]` | $2$ |
| getSampleStats | `[]` | $1$ |
| getBinCount | `[]` | $1$ |
| getBinStats | `[]` | $1$ |
| getUpperBounds | `[]` | $2$ |
| getGeneratorUpperBounds | `[]` | $1$ |
| isLoaded | `[]` | $1$ |
| reSeed | `[long seed]` | $1$ |
| probability | `[double x]` | $1$ |
| density | `[double x]` | $2$ |
| cumulativeProbability | `[double x]` | $8$ |
| inverseCumulativeProbability | `[final double p]` | $6$ |
| getNumericalMean | `[]` | $1$ |
| getNumericalVariance | `[]` | $1$ |
| getSupportLowerBound | `[]` | $1$ |
| getSupportUpperBound | `[]` | $1$ |
| isSupportLowerBoundInclusive | `[]` | $1$ |
| isSupportUpperBoundInclusive | `[]` | $1$ |
| isSupportConnected | `[]` | $1$ |
| reseedRandomGenerator | `[long seed]` | $1$ |
| pB | `[int i]` | $1$ |
| pBminus | `[int i]` | $1$ |
| kB | `[int i]` | $1$ |
| k | `[double x]` | $1$ |
| cumBinP | `[int binIndex]` | $1$ |
| getKernel | `[SummaryStatistics bStats]` | $3$ |
| **Total CC** || **$52$** |



# The class 'NormalizedRandomGenerator'
Package: `org.apache.commons.math3.random.NormalizedRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextNormalizedDouble | `[]` | $1$ |
| **Total CC** || **$1$** |



# The class 'GaussianRandomGenerator'
Package: `org.apache.commons.math3.random.GaussianRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextNormalizedDouble | `[]` | $1$ |
| **Total CC** || **$1$** |



# The class 'RandomAdaptor'
Package: `org.apache.commons.math3.random.RandomAdaptor`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| createAdaptor | `[RandomGenerator randomGenerator]` | $1$ |
| nextBoolean | `[]` | $1$ |
| nextBytes | `[byte[] bytes]` | $1$ |
| nextDouble | `[]` | $1$ |
| nextFloat | `[]` | $1$ |
| nextGaussian | `[]` | $1$ |
| nextInt | `[]` | $1$ |
| nextInt | `[int n]` | $1$ |
| nextLong | `[]` | $1$ |
| setSeed | `[int seed]` | $2$ |
| setSeed | `[int[] seed]` | $2$ |
| setSeed | `[long seed]` | $2$ |
| **Total CC** || **$15$** |



# The class 'UniformRandomGenerator'
Package: `org.apache.commons.math3.random.UniformRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextNormalizedDouble | `[]` | $1$ |
| **Total CC** || **$1$** |



# The class 'CorrelatedRandomVectorGenerator'
Package: `org.apache.commons.math3.random.CorrelatedRandomVectorGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| getGenerator | `[]` | $1$ |
| getRank | `[]` | $1$ |
| getRootMatrix | `[]` | $1$ |
| nextVector | `[]` | $4$ |
| **Total CC** || **$7$** |



# The class 'Well1024a'
Package: `org.apache.commons.math3.random.Well1024a`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'SynchronizedRandomGenerator'
Package: `org.apache.commons.math3.random.SynchronizedRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[int[] seed]` | $1$ |
| setSeed | `[long seed]` | $1$ |
| nextBytes | `[byte[] bytes]` | $1$ |
| nextInt | `[]` | $1$ |
| nextInt | `[int n]` | $1$ |
| nextLong | `[]` | $1$ |
| nextBoolean | `[]` | $1$ |
| nextFloat | `[]` | $1$ |
| nextDouble | `[]` | $1$ |
| nextGaussian | `[]` | $1$ |
| **Total CC** || **$11$** |



# The class 'Well512a'
Package: `org.apache.commons.math3.random.Well512a`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'HaltonSequenceGenerator'
Package: `org.apache.commons.math3.random.HaltonSequenceGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextVector | `[]` | $3$ |
| scramble | `[final int i, final int j, final int b, final int digit]` | $1$ |
| skipTo | `[final int index]` | $1$ |
| getNextIndex | `[]` | $1$ |
| **Total CC** || **$6$** |



# The class 'RandomDataImpl'
Package: `org.apache.commons.math3.random.RandomDataImpl`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| getDelegate | `[]` | $1$ |
| nextHexString | `[int len]` | $1$ |
| nextInt | `[int lower, int upper]` | $1$ |
| nextLong | `[long lower, long upper]` | $1$ |
| nextSecureHexString | `[int len]` | $1$ |
| nextSecureInt | `[int lower, int upper]` | $1$ |
| nextSecureLong | `[long lower, long upper]` | $1$ |
| nextPoisson | `[double mean]` | $1$ |
| nextGaussian | `[double mu, double sigma]` | $1$ |
| nextExponential | `[double mean]` | $1$ |
| nextUniform | `[double lower, double upper]` | $1$ |
| nextUniform | `[double lower, double upper, boolean lowerInclusive]` | $1$ |
| nextBeta | `[double alpha, double beta]` | $1$ |
| nextBinomial | `[int numberOfTrials, double probabilityOfSuccess]` | $1$ |
| nextCauchy | `[double median, double scale]` | $1$ |
| nextChiSquare | `[double df]` | $1$ |
| nextF | `[double numeratorDf, double denominatorDf]` | $1$ |
| nextGamma | `[double shape, double scale]` | $1$ |
| nextHypergeometric | `[int populationSize, int numberOfSuccesses, int sampleSize]` | $1$ |
| nextPascal | `[int r, double p]` | $1$ |
| nextT | `[double df]` | $1$ |
| nextWeibull | `[double shape, double scale]` | $1$ |
| nextZipf | `[int numberOfElements, double exponent]` | $1$ |
| reSeed | `[long seed]` | $1$ |
| reSeedSecure | `[]` | $1$ |
| reSeedSecure | `[long seed]` | $1$ |
| reSeed | `[]` | $1$ |
| setSecureAlgorithm | `[String algorithm, String provider]` | $1$ |
| nextPermutation | `[int n, int k]` | $1$ |
| nextSample | `[Collection<?> c, int k]` | $1$ |
| nextInversionDeviate | `[RealDistribution distribution]` | $1$ |
| nextInversionDeviate | `[IntegerDistribution distribution]` | $1$ |
| **Total CC** || **$32$** |



# The class 'UnitSphereRandomVectorGenerator'
Package: `org.apache.commons.math3.random.UnitSphereRandomVectorGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextVector | `[]` | $3$ |
| **Total CC** || **$3$** |



# The class 'Well44497a'
Package: `org.apache.commons.math3.random.Well44497a`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'ValueServer'
Package: `org.apache.commons.math3.random.ValueServer`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| getNext | `[]` | $8$ |
| fill | `[double[] values]` | $2$ |
| fill | `[int length]` | $2$ |
| computeDistribution | `[]` | $1$ |
| computeDistribution | `[int binCount]` | $1$ |
| getMode | `[]` | $1$ |
| setMode | `[int mode]` | $1$ |
| getValuesFileURL | `[]` | $1$ |
| setValuesFileURL | `[String url]` | $1$ |
| setValuesFileURL | `[URL url]` | $1$ |
| getEmpiricalDistribution | `[]` | $1$ |
| resetReplayFile | `[]` | $2$ |
| closeReplayFile | `[]` | $2$ |
| getMu | `[]` | $1$ |
| setMu | `[double mu]` | $1$ |
| getSigma | `[]` | $1$ |
| setSigma | `[double sigma]` | $1$ |
| reSeed | `[long seed]` | $1$ |
| getNextDigest | `[]` | $2$ |
| getNextReplay | `[]` | $4$ |
| getNextUniform | `[]` | $1$ |
| getNextExponential | `[]` | $1$ |
| getNextGaussian | `[]` | $1$ |
| **Total CC** || **$38$** |



# The class 'RandomGeneratorFactory'
Package: `org.apache.commons.math3.random.RandomGeneratorFactory`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| createRandomGenerator | `[final Random rng]` | $1$ |
| convertToLong | `[int[] seed]` | $2$ |
| **Total CC** || **$3$** |



# The class 'MersenneTwister'
Package: `org.apache.commons.math3.random.MersenneTwister`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $2$ |
| setSeed | `[int[] seed]` | $7$ |
| setSeed | `[long seed]` | $1$ |
| next | `[int bits]` | $4$ |
| **Total CC** || **$14$** |



# The class 'ISAACRandom'
Package: `org.apache.commons.math3.random.ISAACRandom`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[long seed]` | $1$ |
| setSeed | `[int[] seed]` | $4$ |
| next | `[int bits]` | $2$ |
| isaac | `[]` | $3$ |
| isaac2 | `[]` | $1$ |
| isaac3 | `[]` | $1$ |
| initState | `[]` | $5$ |
| shuffle | `[]` | $1$ |
| setState | `[int start]` | $1$ |
| **Total CC** || **$20$** |



# The class 'UncorrelatedRandomVectorGenerator'
Package: `org.apache.commons.math3.random.UncorrelatedRandomVectorGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextVector | `[]` | $2$ |
| **Total CC** || **$2$** |



# The class 'StableRandomGenerator'
Package: `org.apache.commons.math3.random.StableRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextNormalizedDouble | `[]` | $12$ |
| **Total CC** || **$12$** |



# The class 'RandomData'
Package: `org.apache.commons.math3.random.RandomData`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| nextHexString | `[int len]` | $1$ |
| nextInt | `[int lower, int upper]` | $1$ |
| nextLong | `[long lower, long upper]` | $1$ |
| nextSecureHexString | `[int len]` | $1$ |
| nextSecureInt | `[int lower, int upper]` | $1$ |
| nextSecureLong | `[long lower, long upper]` | $1$ |
| nextPoisson | `[double mean]` | $1$ |
| nextGaussian | `[double mu, double sigma]` | $1$ |
| nextExponential | `[double mean]` | $1$ |
| nextUniform | `[double lower, double upper]` | $1$ |
| nextUniform | `[double lower, double upper, boolean lowerInclusive]` | $1$ |
| nextPermutation | `[int n, int k]` | $1$ |
| nextSample | `[Collection<?> c, int k]` | $1$ |
| **Total CC** || **$13$** |



# The class 'BitsStreamGenerator'
Package: `org.apache.commons.math3.random.BitsStreamGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[int[] seed]` | $1$ |
| setSeed | `[long seed]` | $1$ |
| next | `[int bits]` | $1$ |
| nextBoolean | `[]` | $1$ |
| nextDouble | `[]` | $1$ |
| nextFloat | `[]` | $1$ |
| nextGaussian | `[]` | $4$ |
| nextInt | `[]` | $1$ |
| nextInt | `[int n]` | $4$ |
| nextLong | `[]` | $1$ |
| nextLong | `[long n]` | $3$ |
| clear | `[]` | $1$ |
| nextBytes | `[byte[] bytes]` | $1$ |
| nextBytes | `[byte[] bytes, int start, int len]` | $3$ |
| nextBytesFill | `[byte[] bytes, int start, int len]` | $6$ |
| **Total CC** || **$31$** |



# The class 'Well44497b'
Package: `org.apache.commons.math3.random.Well44497b`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| next | `[final int bits]` | $1$ |
| **Total CC** || **$1$** |



# The class 'JDKRandomGenerator'
Package: `org.apache.commons.math3.random.JDKRandomGenerator`

| Method | Parameters | CC |
| ------ | ---------- | -- |
| setSeed | `[int seed]` | $1$ |
| setSeed | `[int[] seed]` | $1$ |
| **Total CC** || **$2$** |



# Histogram of CC values per class
> Total CC of the whole project: 407
The values per class are the sum of the CC values of all methods in the class.
```
 RandomDataGenerator : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (74)
EmpiricalDistribu... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (52)
         ValueServer : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (38)
      RandomDataImpl : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (32)
 BitsStreamGenerator : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (31)
SobolSequenceGene... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (23)
AbstractRandomGen... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (21)
         ISAACRandom : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (20)
       RandomAdaptor : ∎∎∎∎∎∎∎∎∎∎∎ (15)
     MersenneTwister : ∎∎∎∎∎∎∎∎∎∎ (14)
          RandomData : ∎∎∎∎∎∎∎∎∎∎ (13)
StableRandomGener... : ∎∎∎∎∎∎∎∎∎ (12)
     RandomGenerator : ∎∎∎∎∎∎∎∎ (11)
SynchronizedRando... : ∎∎∎∎∎∎∎∎ (11)
CorrelatedRandomV... : ∎∎∎∎∎∎ (7)
        AbstractWell : ∎∎∎∎∎∎ (7)
HaltonSequenceGen... : ∎∎∎∎∎ (6)
UnitSphereRandomV... : ∎∎∎ (3)
RandomGeneratorFa... : ∎∎∎ (3)
UncorrelatedRando... : ∎∎ (2)
  JDKRandomGenerator : ∎∎ (2)
GaussianRandomGen... : ∎∎ (1)
            Well512a : ∎∎ (1)
          Well19937a : ∎∎ (1)
          Well19937c : ∎∎ (1)
NormalizedRandomG... : ∎∎ (1)
          Well44497b : ∎∎ (1)
          Well44497a : ∎∎ (1)
RandomVectorGener... : ∎∎ (1)
UniformRandomGene... : ∎∎ (1)
           Well1024a : ∎∎ (1)
```
*This output is printed as markdown format for readability, you can save to markdown using the `>` bash operator*

