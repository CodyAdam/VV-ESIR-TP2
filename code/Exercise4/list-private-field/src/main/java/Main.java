import java.io.File;
import java.io.IOException;

import com.github.javaparser.utils.SourceRoot;

public class Main {

    public static void main(String[] args) throws IOException {
        if (args.length == 0) {
            System.err.println("Should provide the path to the source code");
            System.exit(1);
        }

        String target = args[0];

        File file = new File(target);
        if (!file.exists() || !file.isDirectory() || !file.canRead()) {
            System.err.println("Provide a path to an existing readable directory");
            System.exit(2);
        }

        System.out.println("# Report list private fields\n");
        System.out.println("Looking for public classes without getters in `" + target + "`");

        SourceRoot root = new SourceRoot(file.toPath());
        NoGetterPrinter printer = new NoGetterPrinter();
        root.parse("", (localPath, absolutePath, result) -> {
            result.ifSuccessful(unit -> unit.accept(printer, null));
            return SourceRoot.Callback.Result.DONT_SAVE;
        });
        System.out.println("*This output is print as markdown format for readability*\n");
    }

}
