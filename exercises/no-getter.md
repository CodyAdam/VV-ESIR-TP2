# No getter!

With the help of JavaParser implement a program that obtains the private fields of public classes that have no public getter in a Java project. 

A field has a public getter if, in the same class, there is a public method that simply returns the value of the field and whose name is `get<name-of-the-field>`.

For example, in the following class:

```Java

class Person {
    private int age;
    private String name;
    
    public String getName() { return name; }

    public boolean isAdult() {
        return age > 17;
    }
}
```

`name` has a public getter, while `age` doesn't.

The program should take as input the path to the source code of the project. It should produce a report in the format of your choice (TXT, CSV, Markdown, HTML, etc.) that lists for each detected field: its name, the name of the declaring class and the package of the declaring class.

Include in this repository the code of your application. Remove all unnecessary files like compiled binaries. See the [instructions](../sujet.md) for suggestions on the projects to use.

*Disclaimer* In a real project not all fields need to be accessed with a public getter.

# Solution

The solution is available in the [Exercise4](../code/Exercise4/) folder.

-   Running the program on the __Apache Commons Math project__
    We founded a total of **2792 private fields without public getter**.


    The full report is available [here](../code/Exercise4/long-report.md).


-   Running the program on the __Apache Commons Math Random package__ (smaller sample)
    We founded a total of **113 private fields without public getter**.

    The full report is available [here](../code/Exercise4/report.md).

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
java -jar list-private-fields-1.0-jar-with-dependencies.jar <path-to-source-directory>
```

## How to preview the report

The report is outputed in Markdown. To preview it you can use the bash `>` operator to write the output to a file and then use a Markdown previewer.

```bash
java -jar list-private-fields-1.0-jar-with-dependencies.jar <path-to-source-directory> > report.md
```

