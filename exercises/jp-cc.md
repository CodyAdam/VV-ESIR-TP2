# Cyclomatic Complexity with JavaParser

With the help of JavaParser implement a program that computes the Cyclomatic Complexity (CC) of all methods in a given Java project. The program should take as input the path to the source code of the project. It should produce a report in the format of your choice (TXT, CSV, Markdown, HTML, etc.) containing a table showing for each method: the package and name of the declaring class, the name of the method, the types of the parameters and the value of CC.
Your application should also produce a histogram showing the distribution of CC values in the project. Compare the histogram of two or more projects.


Include in this repository the code of your application. Remove all unnecessary files like compiled binaries. Do include the reports and plots you obtained from different projects. See the [instructions](../sujet.md) for suggestions on the projects to use.

You may use [javaparser-starter](../code/javaparser-starter) as a starting point.

*This exercise grants bonus points :)*


# Solution



The solution is available in the [Exercice6](/code/Exercise6/) folder.

- You can find a sample report [here](/code/Exercise6/report.md). This sample was generated on the examples classes in the [Exercise5/get-tcc/examples](/code/Exercise5/get-tcc/examples/) directory.
- You can find another report [here](/code/Exercise6/common-math-random-report.md). This sample was generated on the [Apache Commons Math](https://github.com/apache/commons-math) Random Package source (smaller sample).
- You can find another report [here](/code/Exercise6/common-math-report.md). This sample was generated on the [Apache Commons Math](https://github.com/apache/commons-math) source (**full**).
- You can find another report [here](/code/Exercise6/commons-cli-master-report.md). This sample was generated on the [Apache Commons CLI](https://github.com/apache/commons-cli) source (**full**).

# Comparison of the results

## Apache Commons Math

### Histogram of CC values per class

[full report](/code/Exercise6/common-math-report.md)

> Total CC of the whole project: 25212
The values per class are the sum of the CC values of all methods in the class.
```
            FastMath : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (591)
                 Dfp : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (370)
        FastMathTest : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (316)
DerivativeStructu... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (284)
RealVectorAbstrac... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (239)
          DSCompiler : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (217)
MillerUpdatingReg... : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (195)
       FieldRotation : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (192)
     BOBYQAOptimizer : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (189)
          MathArrays : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (186)
     BlockRealMatrix : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (175)

              + 1 405 more...
```

## Apache Commons CLI

### Histogram of CC values per class

[full report](/code/Exercise6/commons-cli-master-report.md)
> Total CC of the whole project: 926
The values per class are the sum of the CC values of all methods in the class.
```
       DefaultParser : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (131)
       HelpFormatter : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (92)
      ParserTestCase : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (66)
              Option : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (59)
              Parser : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (54)
         CommandLine : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (45)
         PosixParser : ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎ (43)
     BasicParserTest : ∎∎∎∎∎∎∎∎∎∎∎ (27)
   HelpFormatterTest : ∎∎∎∎∎∎∎∎∎∎∎ (27)
           GnuParser : ∎∎∎∎∎∎∎∎∎∎∎ (25)
             Options : ∎∎∎∎∎∎∎∎∎∎∎ (25)

                + 40 more...
```

The maximum "class CC value" is 131 for the Commons ClI project which does not even compete with the top 10 classes of the Common Math project which are above 170 of class CC score. We should still take into consideration that the Common Math project is much bigger than the Apache Commons CLI project.

## How to build 

(require java 11)
To build the project, run the following command:

```bash
mvn clean package
```

The jar file will be generated in the `target` folder.

## How to run

To run the project on a source directory, run the following command:

```bash
java -jar get-cc-1.0-jar-with-dependencies.jar <path-to-source-directory>
```

## How to preview the report

The report is outputed in Markdown. To preview it you can use the bash `>` operator to write the output to a file and then use a Markdown previewer.

```bash
java -jar get-cc-1.0-jar-with-dependencies.jar <path-to-source-directory> > report.md
```