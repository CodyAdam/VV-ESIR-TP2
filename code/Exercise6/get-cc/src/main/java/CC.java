import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.Node;
import com.github.javaparser.ast.body.ClassOrInterfaceDeclaration;
import com.github.javaparser.ast.body.MethodDeclaration;
import com.github.javaparser.ast.body.TypeDeclaration;
import com.github.javaparser.ast.stmt.Statement;
import com.github.javaparser.ast.visitor.VoidVisitorWithDefaults;

// This class visits a compilation unit and
// prints all public enum, classes or interfaces along with their public methods
public class CC extends VoidVisitorWithDefaults<Void> {

    static Map<String, Integer> ccMap = new HashMap<String, Integer>();

    @Override
    public void visit(CompilationUnit unit, Void arg) {
        for (TypeDeclaration<?> type : unit.getTypes()) {
            type.accept(this, null);
        }
    }

    public static Map<String, Integer> getCcMap() {
        return ccMap;
    }

    /**
     * It should produce a report in the Markdown format containing a table showing
     * for each method: the package and name of the declaring class, the name of the
     * method, the types of the parameters and the value of CC.
     */
    @Override
    public void visit(ClassOrInterfaceDeclaration declaration, Void arg) {
        // Nice output in markdown format
        System.out.println(
                "\n# The class '" + declaration.getNameAsString() + "'");
        System.out.println("Package: `" + declaration.getFullyQualifiedName().get() + "`");

        // Header of the table
        System.out.println(
                "\n| Method | Parameters | CC |");
        System.out.println(
                "| ------ | ---------- | -- |");
        int classCc = 0;
        // Go through all methods
        for (MethodDeclaration method : declaration.getMethods()) {
            int cc = getCC(method);

            // Nice output in markdown format
            System.out.print(
                    "| " + method.getNameAsString());
            System.out.print(
                    " | `" + method.getParameters());
            System.out.println(
                    "` | $" + cc + "$ |");
            classCc += cc;
        }

        // class total cc print
        System.out.println(
                "| **Total CC** || **$" + classCc + "$** |");

        // Add the class to the map
        ccMap.put(declaration.getNameAsString(), classCc);

        System.out.println("\n");
    }

    /**
     * The Cyclomatic complexity (CC), V (G) for a flow graph G can be defined as
     * V (G) = E - N + 2
     * Where: E is total number of edges in the flow graph.
     * N is the total number of nodes in the flow graph.
     */
    private int getCC(MethodDeclaration method) {
        if (method.getBody().isEmpty()) {
            return 1;
        }
        int cc = 1 + getCc(method.getBody().get());
        return cc;
    }

    /**
     * Assign one point to account for the start of the method
     * Add one point for each conditional construct, such as an "if" condition
     * Add one point for each iterative structure
     * Add one point for each case or default block in a switch statement
     * Add one point for any additional boolean condition,
     * such as the use of && or ||
     */
    private int getCc(Statement node) {
        int cc = 0;

        if (node.isIfStmt()) {
            cc++;
            // foreach else/elseif statement add 1
            if (node.asIfStmt().getElseStmt().isPresent()) {
                cc+= node.asIfStmt().getElseStmt().get().getChildNodes().size();
            }
        }
        if (node.isWhileStmt()) {
            cc++;
        }
        if (node.isForStmt()) {
            cc++;
        }
        if (node.isForEachStmt()) {
            cc++;
        }
        if (node.isDoStmt()) {
            cc++;
        }
        if (node.isSwitchStmt()) {
            // foreach case statement add 1
            cc += node.asSwitchStmt().getEntries().size();
        }
        List<Node> childs = node.getChildNodes();
        for (Node child : childs) {
            if (child instanceof Statement) {
                cc += getCc((Statement) child);
            }
        }
        return cc;
    }
}