# TCC *vs* LCC

Explain under which circumstances *Tight Class Cohesion* (TCC) and *Loose Class Cohesion* (LCC) metrics produce the same value for a given Java class. Build an example of such as class and include the code below or find one example in an open-source project from Github and include the link to the class below. Could LCC be lower than TCC for any given class? Explain.

## Answer

TCC and LCC will produce the same value for a given Java class if : 

- all public methods are connected to each other (i.e. they access the same class-level variable)
- there is no indirect connection between public methods 

Exemple of a class that will produce the same value for TCC and LCC :

```java
public class MyClass {
    int x = 0;
    public void method1() {
        x += 1;
    }

    public void method2() {
        x += 2;
    }

    public void method3() {
        x += 3;
    }
}
```
In the example above, all public methods are connected to each other and there is no indirect connection between public methods. Therefore, TCC and LCC will produce the same value for this class.

```plantuml
object methode1
object methode2
object methode3


methode1 - methode2 : x 
methode2 - methode3 : x 
methode3 - methode1 : x
```

Calculating TCC and LCC for this class will produce the same value :

$$ 
max\_connection = \frac{N * (N-1)}{2} = \frac{3 * (3-1)}{2} = 3
$$
$$
TCC = \frac{direct\_connection}{max\_connection} = \frac{3}{3} = 1
$$
$$
LCC = \frac{direct\_connection+indirect\_connection}{max\_connection} = \frac{3 + 0}{3} = 1 = TCC
$$


### Case LCC lower than TCC

It is theoretically impossible to have LCC lower than TCC for any given class, because LCC is a subset of TCC. LCC is the number of direct and indirect connections between methods, while TCC is the number of direct connections between methods. So LCC will always be equal or higher than TCC.