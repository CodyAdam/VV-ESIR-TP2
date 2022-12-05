# Class cohesion with JavaParser

With the help of JavaParser implement a program that computes the Tight Class Cohesion (TCC) for each class in a given Java project. The program should take as input the path to the source code of the project. It should produce a report in the format of your choice (TXT, CSV, Markdown, HTML, etc.) containing a table showing for each class: the package, name and TCC value. 
Your application should also produce a histogram showing the distribution of CC values in the project. Compare the histogram of two or more projects.
Finally, your application should also produce the dependency graph of each class (cf. example [here](https://people.irisa.fr/Benoit.Combemale/pub/course/vv/vv-textbook-v0.1.pdf#cohesion-graph)). The graph should be written using the [GraphViz DOT format](https://www.graphviz.org/)

Ignore inherited members to compute TCC of a class.

Include in this repository the code of your application. Remove all unnecessary files like compiled binaries. Do include the reports and plots you obtained from different projects. See the [instructions](../sujet.md) for suggestions on the projects to use.

You may use [javaparser-starter](../code/javaparser-starter) as a starting point.


# Solution

The solution is available in the [Exercice5](/code/Exercise5/) folder.

You can find a sample report [here](/code/Exercise5/report.md). This sample was generated on the examples classes in the [Exercise5/get-tcc/examples](/code/Exercise5/get-tcc/examples/) directory.

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
java -jar get-tcc-1.0-jar-with-dependencies.jar <path-to-source-directory>
```

## How to preview the report

The report is outputed in Markdown. To preview it you can use the bash `>` operator to write the output to a file and then use a Markdown previewer.

```bash
java -jar get-tcc-1.0-jar-with-dependencies.jar <path-to-source-directory> > report.md
```
