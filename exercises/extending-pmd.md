# Extending PMD

Use XPath to define a new rule for PMD to prevent complex code. The rule should detect the use of three or more nested `if` statements in Java programs so it can detect patterns like the following:

```Java
if (...) {
    ...
    if (...) {
        ...
        if (...) {
            ....
        }
    }

}
```
Notice that the nested `if`s may not be direct children of the outer `if`s. They may be written, for example, inside a `for` loop or any other statement.
Write below the XML definition of your rule.

You can find more information on extending PMD in the following link: https://pmd.github.io/latest/pmd_userdocs_extending_writing_rules_intro.html, as well as help for using `pmd-designer` [here](https://github.com/selabs-ur1/VV-TP2/blob/master/exercises/designer-help.md).

Use your rule with different projects and describe you findings below. See the [instructions](../sujet.md) for suggestions on the projects to use.

## Answer

### Rule that find more than 3 nested if statements 

The create rule is the following :

```xml
<?xml version="1.0"?>

<ruleset name="Custom Rules"
  xmlns="http://pmd.sourceforge.net/ruleset/2.0.0"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://pmd.sourceforge.net/ruleset/2.0.0 https://pmd.sourceforge.io/ruleset_2_0_0.xsd">
  <description>
        My custom rules
  </description>

  <rule name="TooMuchNestedIfStatement"
    language="java"
    message="You should avoid nesting more than 2 if statement"
    class="net.sourceforge.pmd.lang.rule.XPathRule">
    <description></description>
    <priority>4</priority>
    <properties>
      <property name="version" value="2.0" />
      <property name="xpath">
        <value>
          <![CDATA[
/IfStatement[count(ancestor::BlockStatement/Statement/IfStatement)>=3]
]]>
        </value>
      </property>
    </properties>
  </rule>
</ruleset>
```

You can find the ruleset file [here](../code/Exercise3/rules/my-ruleset.xml) 

### Test the rule

To test the rule, we use the [source from exercice 2 (Apache Common Math)](../code/Exercise2/source/commons-math3-3.6.1-src.zip)

We founded **355 issues** of more than 3 nested if statements. (The output of PMD is available [here](../code/Exercise3/output.txt))

### Analyse the results

Let's analyse the first issue founded by PMD :

```
/home/cody/Git/VV-ESIR-TP2/code/Exercise2/source/commons-math3-3.6.1-src/src/main/java/org/apache/commons/math3/analysis/polynomials/PolynomialFunction.java:321:
	TooMuchNestedIfStatement:	You should avoid nesting more than 2 if statement
```

Here is the code related to the issue :


```java
    public String toString() {
        StringBuilder s = new StringBuilder();
        if (coefficients[0] == 0.0) {
            if (coefficients.length == 1) {
                return "0";
            }
        } else {
            s.append(toString(coefficients[0]));
        }

        for (int i = 1; i < coefficients.length; ++i) {
            if (coefficients[i] != 0) {
                if (s.length() > 0) {
                    if (coefficients[i] < 0) { // <-- third nested if (1)
                        s.append(" - ");
                    } else {
                        s.append(" + ");
                    }
                } else {
                    if (coefficients[i] < 0) { // <-- third nested if (2)
                        s.append("-");
                    }
                }

                double absAi = FastMath.abs(coefficients[i]);
                if ((absAi - 1) != 0) {
                    s.append(toString(absAi));
                    s.append(' ');
                }

                s.append("x");
                if (i > 1) {
                    s.append('^');
                    s.append(Integer.toString(i));
                }
            }
        }

        return s.toString();
    }
```

On the second marked if statement we can observe that the developer choose to use a `else` statement instead of a `else if` statement. This may be a choice made by the developer team to improve the readability of the code.

Let's try to refactor this code to avoid the third nested if statement.

```java
    public String toString() {
        StringBuilder s = new StringBuilder();
        if (coefficients[0] == 0.0) {
            if (coefficients.length == 1) {
                return "0";
            }
        } else {
            s.append(toString(coefficients[0]));
        }

        for (int i = 1; i < coefficients.length; ++i) {
            if (coefficients[i] != 0) {
                if (s.length() > 0 && coefficients[i] < 0) {
                    s.append(" - ");
                } else if (s.length() > 0) {
                    s.append(" + ");
                } else if (coefficients[i] < 0) {
                    s.append("-");
                }

                double absAi = FastMath.abs(coefficients[i]);
                if ((absAi - 1) != 0) {
                    s.append(toString(absAi));
                    s.append(' ');
                }

                s.append("x");
                if (i > 1) {
                    s.append('^');
                    s.append(Integer.toString(i));
                }
            }
        }

        return s.toString();
    }
```
Here we can see that the behaviour of the code is harder to read because of the more complex condition and the duplicated `s.length() < 0` condition which make the behavious of the `if` harder to think about (in my opinion).

### Conclusion

The rule we created had the purpose to detect complex code to improve the readability of the code. But in this case, the code is not complex and quite clear as it is.

So, we can conclude that rule are good to have but they are not always pertinent depending on the context.