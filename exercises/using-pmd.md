# Using PMD

Pick a Java project from Github (see the [instructions](../sujet.md) for suggestions). Run PMD on its source code using any ruleset. Describe below an issue found by PMD that you think should be solved (true positive) and include below the changes you would add to the source code. Describe below an issue found by PMD that is not worth solving (false positive). Explain why you would not solve this issue.

## Answer


For this exercise, I used the [Apache Commons Math](https://github.com/apache/commons-math).

The rulesets I used and the output of PMD are available [here](/code/Exercise2/).

### True positive


One of the issue I found was about code bestpractice. Here is the PMD output : 

```
commons-math3-3.6.1-src/src/main/java/org/apache/commons/math3/optimization/direct/BOBYQAOptimizer.java:1839:
OneDeclarationPerLine:	Use one line for each declaration, it enhances code readability.
```

The code in question is the following near line 1839 : 

```java
  private double[] trsbox(
    double delta,
    ArrayRealVector gnew,
    ArrayRealVector xbdi,
    ArrayRealVector s,
    ArrayRealVector hs,
    ArrayRealVector hred
  ) {
    printMethod(); // XXX

    final int n = currentBest.getDimension();
    final int npt = numberOfInterpolationPoints;

    double dsq = Double.NaN;
    double crvmin = Double.NaN;

    // Local variables
    double ds;
    int iu;
    double dhd, dhs, cth, shs, sth, ssq, beta=0, sdec, blen;
    int iact = -1;
    int nact = 0;
    double angt = 0, qred;
    int isav;
    double temp = 0, xsav = 0, xsum = 0, angbd = 0, dredg = 0, sredg = 0;
    int iterc;
    double resid = 0, delsq = 0, ggsav = 0, tempa = 0, tempb = 0,
    redmax = 0, dredsq = 0, redsav = 0, gredsq = 0, rednew = 0;
    int itcsav = 0;
    double rdprev = 0, rdnext = 0, stplen = 0, stepsq = 0;
    int itermax = 0;

    // [...] the rest of the code of the method
  }
```

I labeled this issue as true positive but, in my opinion, this case is in the middle between true positive and false positive which I founded to be interesting. It is a good practice to have one declaration per line, but here, we have a total of 39 variables declared in this method. Which may be easier to read if they are declared on the same line to keep code clear and short.

But this code raises another problem : **it's not consistent**. either we declare all the variables on the same line, or we declare them on different lines. But here, we have a mix of both... And it is not even grouped/sorted by type.

here are the changes I would make to the code (2 options) : 

- Use one line for each declaration as the PMD output suggests and sort by type

```java
  private double[] trsbox(
    double delta,
    ArrayRealVector gnew,
    ArrayRealVector xbdi,
    ArrayRealVector s,
    ArrayRealVector hs,
    ArrayRealVector hred
  ) {
    printMethod(); // XXX

    final int n = currentBest.getDimension();
    final int npt = numberOfInterpolationPoints;

    double dsq = Double.NaN;
    double crvmin = Double.NaN;

    // Local variables
    double ds;
    double dhd;
    double dhs;
    double cth;
    double shs;
    double sth;
    double ssq;
    double sdec;
    double blen;
    double qred;
    double beta=0;
    double angt = 0;
    double temp = 0;
    double xsav = 0;
    double xsum = 0;
    double angbd = 0;
    double dredg = 0;
    double sredg = 0;
    double resid = 0
    double delsq = 0;
    double ggsav = 0;
    double tempa = 0;
    double tempb = 0;
    double rdprev = 0;
    double rdnext = 0;
    double stplen = 0;
    double stepsq = 0;
    int iu;
    int isav;
    int iterc;
    int iact = -1;
    int nact = 0;
    int itcsav = 0;
    int itermax = 0;
    redmax = 0;
    dredsq = 0;
    redsav = 0;
    gredsq = 0;
    rednew = 0;

    // [...] the rest of the code of the method
  }
```

- Use a consistent way to declare variables. We declare all the variables on the same line and group them by type and initialization. For this big amount of variable, it could improve readability.

```java
  private double[] trsbox(
    double delta,
    ArrayRealVector gnew,
    ArrayRealVector xbdi,
    ArrayRealVector s,
    ArrayRealVector hs,
    ArrayRealVector hred
  ) {
    printMethod(); // XXX

    final int n = currentBest.getDimension();
    final int npt = numberOfInterpolationPoints;

    double dsq = Double.NaN;
    double crvmin = Double.NaN;

    // Local variables
    double ds, dhd, dhs, cth, shs, sth, ssq, sdec, blen, qred;
    
    double beta=0, angt = 0, temp = 0, xsav = 0, xsum = 0, angbd = 0, dredg = 0, sredg = 0, resid = , delsq = 0, ggsav = 0, tempa = 0, tempb = 0, rdprev = 0, rdnext = 0, stplen = 0, stepsq = 0;
    
    int iu, isav, iterc
    
    int iact = -1, nact = 0, itcsav = 0, itermax = 0;

    redmax = 0, dredsq = 0, redsav = 0, gredsq = 0, rednew = 0;

    // [...] the rest of the code of the method
  }
```

### False positive


The other issue is the following :


```
commons-math3-3.6.1-src/src/main/java/org/apache/commons/math3/distribution/KolmogorovSmirnovDistribution.java:280:
UnusedAssignment:	The initializer for variable 'h' is never used (overwritten on lines 283, 286 and 288)
```

Here is the related code :

```java
  private FieldMatrix<BigFraction> createH(double d)
          throws NumberIsTooLargeException, FractionConversionException {

    int k = (int) FastMath.ceil(n * d);

    int m = 2 * k - 1;
    double hDouble = k - n * d;

    if (hDouble >= 1) {
        throw new NumberIsTooLargeException(hDouble, 1.0, false);
    }

    BigFraction h = null; // line 280 (target of the issue)

    try {
        h = new BigFraction(hDouble, 1.0e-20, 10000); // line 283
    } catch (FractionConversionException e1) {
        try {
            h = new BigFraction(hDouble, 1.0e-10, 10000); // line 286
        } catch (FractionConversionException e2) {
            h = new BigFraction(hDouble, 1.0e-5, 10000); // line 288
        }
    }

    // [...] the rest of the code of the method
    // h is used 3 times after this block
  }
```

PMD is telling us that "The initializer for variable 'h' is never used (overwritten on lines 283, 286 and 288)" which means that the variable `h` got assigned the value of `null` but this value is never used (overwritten on lines 283, 286 and 288).

PMD is suggesting replacing the line 280 by `BigFraction h;` without any initialization.

By default in java, if no initialization value is given to a variable which is not primitive typed, it is initialized to null.

I consider this issue as a false positive because the variable `h` is initialized to `null` by default and the code is more readable with the initialization to `null` than without it. Furthermore, initialize variables upon definition is considered as a good practice.