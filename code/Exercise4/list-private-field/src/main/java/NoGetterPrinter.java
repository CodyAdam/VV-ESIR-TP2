import java.util.HashSet;
import java.util.Set;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.body.*;
import com.github.javaparser.ast.visitor.VoidVisitorWithDefaults;

// This class visits a compilation unit and
// prints all public enum, classes or interfaces along with their public methods
public class NoGetterPrinter extends VoidVisitorWithDefaults<Void> {

    @Override
    public void visit(CompilationUnit unit, Void arg) {
        for (TypeDeclaration<?> type : unit.getTypes()) {
            type.accept(this, null);
        }
    }

    @Override
    public void visit(ClassOrInterfaceDeclaration declaration, Void arg) {
        if (!declaration.isPublic()) // We only care about public classes
            return;

        // Get all getters methods in a set of string
        Set<String> getters = new HashSet<>();
        for (MethodDeclaration method : declaration.getMethods()) {
            if (method.isPublic()) {
                String fullMethodName = method.getDeclarationAsString();
                if (fullMethodName.length() >= 4 && fullMethodName.startsWith("get")) {
                    String name = method.getNameAsString().substring(3);
                    // lower case the first letter
                    name = name.substring(0, 1).toLowerCase() + name.substring(1);
                    getters.add(name);
                }
            }
        }

        boolean firstPass = true;
        // Check private fields without getter
        for (BodyDeclaration<?> member : declaration.getMembers()) {
            if (member instanceof FieldDeclaration) {
                FieldDeclaration field = (FieldDeclaration) member;
                if (field.isPrivate()) {
                    String name = field.getVariable(0).getNameAsString();
                    if (!getters.contains(name)) {
                        if (firstPass) {
                            String fullName = declaration.getFullyQualifiedName().orElse("[Anonymous]");
                            // only part after the last dot
                            System.out.println("\n\n## In " + fullName.substring(fullName.lastIndexOf('.') + 1) + "\n");
                            System.out.println("Full name: `" + fullName + "`");
                            System.out.println("Path: `"
                                    + declaration.findCompilationUnit().get().getStorage().get().getPath() + "`\n");
                            firstPass = false;
                        }
                        int line = field.getBegin().get().line;
                        System.out.println("- `" + name + "`" + " (line " + line + ")");
                    }
                }
            }
        }
    }
}
