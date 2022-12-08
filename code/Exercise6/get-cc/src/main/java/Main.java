import java.io.File;
import java.io.IOException;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

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

        System.out.println("# Report CC Value\n");
        System.out.println("Calculating the CC values of the classes in `" + target + "`");

        System.out.println("# [Histogram of CC values per class](#histogram-of-cc-values-per-class)");

        SourceRoot root = new SourceRoot(file.toPath());
        CC visitor = new CC();
        root.parse("", (localPath, absolutePath, result) -> {
            result.ifSuccessful(unit -> {
                unit.accept(visitor, null);
            });
            return SourceRoot.Callback.Result.DONT_SAVE;
        });

        // Print the total CC for the whole project
        Map<String, Integer> ccMap = CC.getCcMap();

        int totalCc = 0;
        int max = 0;
        for (Map.Entry<String, Integer> entry : ccMap.entrySet()) {
            totalCc += entry.getValue();
            if (entry.getValue() > max) {
                max = entry.getValue();
            }
        }

        // Print a histogram of the CC values
        // 100% is the maximum CC value make a bar of 50 characters
        System.out.println("\n# Histogram of CC values per class");
        System.out.println("> Total CC of the whole project: " + totalCc);
        System.out.println("The values per class are the sum of the CC values of all methods in the class.");
        System.out.println("```");
        // get keys sorted by value
        List<String> keysSorted = ccMap.entrySet().stream()
                .sorted(Map.Entry.comparingByValue(Comparator.reverseOrder()))
                .map(Map.Entry::getKey)
                .collect(Collectors.toList());
        for (String key : keysSorted) {
            int value = ccMap.get(key);
            // name of the class 20 characters add leading spaces
            String name = key;
            for (int i = 0; i < 20 - name.length(); i++) {
                System.out.print(" ");
            }
            // cut name if too long
            if (name.length() > 20) {
                name = name.substring(0, 17) + "...";
            }
            System.out.print(name + " : ");

            int barLength = (int) Math.round((double) value / max * 50);
            for (int i = 0; i < barLength + 1; i++) {
                System.out.print("∎");
            }
            System.out.println(" (" + value + ")");
        }
        System.out.println("```");

        System.out.println(
                "*This output is printed as markdown format for readability, you can save to markdown using the `>` bash operator*\n");
    }

}
