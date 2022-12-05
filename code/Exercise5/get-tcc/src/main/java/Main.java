import java.io.File;
import java.io.IOException;

import com.github.javaparser.utils.SourceRoot;

public class Main {

    public static void main(String[] args) throws IOException {
        String target;
        if (args.length == 0) {
            System.err.println("Should provide the path to the source code");
            System.exit(1);
        }
        target = args[0];

        File file = new File(target);
        if (!file.exists() || !file.isDirectory() || !file.canRead()) {
            System.err.println("Provide a path to an existing readable directory");
            System.exit(2);
        }

        System.out.println("# Report list private fields\n");
        System.out.println("Calculating the TCC values of the classes in `" + target + "`");

        SourceRoot root = new SourceRoot(file.toPath());
        TCC visitor = new TCC();
        root.parse("", (localPath, absolutePath, result) -> {
            result.ifSuccessful(unit -> {
                unit.accept(visitor, null);
            });
            return SourceRoot.Callback.Result.DONT_SAVE;
        });

        System.out.println(
                "*This output is printed as markdown format for readability, you can save to markdown using the `>` bash operator*\n");
    }

}
