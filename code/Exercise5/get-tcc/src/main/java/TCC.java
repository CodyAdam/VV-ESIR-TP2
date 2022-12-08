import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.body.*;
import com.github.javaparser.ast.expr.MethodCallExpr;
import com.github.javaparser.ast.expr.NameExpr;
import com.github.javaparser.ast.visitor.VoidVisitorWithDefaults;

// This class visits a compilation unit and
// prints all public enum, classes or interfaces along with their public methods
public class TCC extends VoidVisitorWithDefaults<Void> {
    @Override
    public void visit(CompilationUnit unit, Void arg) {
        for (TypeDeclaration<?> type : unit.getTypes()) {
            type.accept(this, null);
        }
    }

    @Override
    public void visit(ClassOrInterfaceDeclaration declaration, Void arg) {
        // Collect information about the methods and attributes of the class.
        Set<MethodDeclaration> methods = new HashSet<>();
        Set<String> attributes = new HashSet<>();
        for (BodyDeclaration<?> member : declaration.getMembers()) {
            if (member instanceof MethodDeclaration) {
                methods.add((MethodDeclaration) member);
            } else if (member instanceof FieldDeclaration) {
                List<VariableDeclarator> variables = ((FieldDeclaration) member).getVariables();
                for (VariableDeclarator variable : variables) {
                    attributes.add(variable.getNameAsString());
                }
            }
        }

        // Nice output in markdown format
        System.out.println(
                "\n# The class '" + declaration.getNameAsString() + "'");
        System.out.println("Package: `" + declaration.getFullyQualifiedName().get() + "`\n");
        System.out.print("methods : [ ");
        for (MethodDeclaration method : methods) {
            System.out.print("`" + method.getNameAsString() + "` ");
        }
        System.out.print("]\n\nattributes : [ ");
        for (String attribute : attributes) {
            System.out.print("`" + attribute + "` ");
        }
        System.out.println("]\n");

        // Check if TCC is 0
        if (methods.size() == 0 || attributes.size() == 0) {
            System.out.println("\nThe TCC value is 0 because the class has no methods or no attributes.");
            System.out.println("\n");
            return;
        }

        // Get all methods attributes uses
        Map<MethodDeclaration, Set<String>> methodToAttributes = new HashMap<>();
        methods.forEach(
                method -> {
                    // get all nameExpr in the method body
                    // and check if they are in the attributes list
                    List<NameExpr> nameExprs = method.findAll(NameExpr.class);
                    for (NameExpr nameExpr : nameExprs) {
                        if (attributes.contains(nameExpr.getNameAsString())) {
                            methodToAttributes.computeIfAbsent(method, k -> new HashSet<>())
                                    .add(nameExpr.getNameAsString());
                        }
                    }
                });
        System.out.println("\nThe attributes used in each method are:");
        for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToAttributes.entrySet()) {
            System.out.println("  - " + entry.getKey().getNameAsString() + " uses " + entry.getValue());
        }

        // Get all methods that call other methods
        Map<MethodDeclaration, Set<String>> methodToCalls = new HashMap<>();
        methods.forEach(
                method -> {
                    // get all methodCallExpr in the method body
                    // and check if they are in the methods list
                    List<MethodCallExpr> methodCallExprs = method.findAll(MethodCallExpr.class);
                    for (MethodCallExpr methodCallExpr : methodCallExprs) {
                        for (MethodDeclaration methodDeclaration : methods) {
                            if (methodDeclaration.getNameAsString().equals(methodCallExpr.getNameAsString())) {
                                methodToCalls.computeIfAbsent(method, k -> new HashSet<>())
                                        .add(methodCallExpr.getNameAsString());
                            }
                        }
                    }
                });

        System.out.println("\nThe methods called in each method are:");
        for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToCalls.entrySet()) {
            System.out.println("  - " + entry.getKey().getNameAsString() + " calls " + entry.getValue());
        }

        double nbMaxPairs = (methods.size() * (methods.size() - 1)) / 2.0;
        double nbDirectConnection = 0;

        // print the mermaid graph of the connections
        System.out.println("\n```mermaid");
        System.out.println("graph TD");

        // calls connections
        for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToCalls.entrySet()) {
            for (String name : entry.getValue()) {
                System.out
                        .println("  " + entry.getKey().getNameAsString() + " -->|call| "
                                + name);
            }
        }

        // attributes connections
        for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToAttributes.entrySet()) {
            for (String attribute : entry.getValue()) {
                System.out.println(
                        "  " + entry.getKey().getNameAsString() + " -. use .->" + attribute + "((" + attribute + "))");
            }
        }

        boolean hasChanged = true;
        while (hasChanged) {
            hasChanged = false;
            // attributes connections through calls
            Map<MethodDeclaration, Set<String>> toAdd = new HashMap<>();
            for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToCalls.entrySet()) {
                for (String name : entry.getValue()) {
                    for (Map.Entry<MethodDeclaration, Set<String>> entry2 : methodToAttributes.entrySet()) {
                        if (entry2.getKey().getNameAsString().equals(name)) {
                            for (String attribute : entry2.getValue()) {
                                toAdd.computeIfAbsent(entry.getKey(), k -> new HashSet<>())
                                        .add(attribute);
                            }
                        }
                    }
                }
            }

            // add toAdd to methodToAttributes
            for (Map.Entry<MethodDeclaration, Set<String>> entry : toAdd.entrySet()) {
                // if the method already has the attribute, don't add it
                if (methodToAttributes.get(entry.getKey()) != null
                        && methodToAttributes.get(entry.getKey()).containsAll(entry.getValue())) {
                    continue;
                }
                hasChanged = true;
                methodToAttributes.computeIfAbsent(entry.getKey(), k -> new HashSet<>())
                        .addAll(entry.getValue());
            }
        }

        // connection through attributes
        for (Map.Entry<MethodDeclaration, Set<String>> entry : methodToAttributes.entrySet()) {
            for (String attribute : entry.getValue()) {
                for (Map.Entry<MethodDeclaration, Set<String>> entry2 : methodToAttributes.entrySet()) {
                    if (entry2.getKey().getNameAsString().equals(entry.getKey().getNameAsString())) {
                        continue;
                    }
                    if (entry2.getValue().contains(attribute)
                            && entry.getKey().getNameAsString().compareTo(entry2.getKey().getNameAsString()) > 0) {
                        System.out.println(
                                "  " + entry.getKey().getNameAsString() + " ----|link by " + attribute + "| "
                                        + entry2.getKey().getNameAsString());
                        nbDirectConnection++;
                    }
                }
            }
        }

        System.out.println("```");
        System.out.println("\nNumber of max pairs: $" + nbMaxPairs + "$\n");
        System.out.println("Number of direct connections (link by): $" + nbDirectConnection + "$\n");
        System.out.println("**TCC value: $" + nbDirectConnection / nbMaxPairs + "$**\n");

        System.out.println("\n");
    }
}
