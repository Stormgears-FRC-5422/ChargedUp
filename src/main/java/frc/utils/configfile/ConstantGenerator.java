package frc.utils.configfile;


import java.io.FileReader;
import java.io.FileWriter;
import java.util.*;
import java.util.stream.Collectors;

public class ConstantGenerator {

    static Map<String, Map<String, propItem>> allConstants = new HashMap<>();

    static Map<String, String> properties = new HashMap<>();

    private static String quote(String s) {
        return "\"" + s + "\"";
    }

    public static void init(String path) throws Exception {
        // where are closing this file
        Object defaultVal;
        FileReader reader = new FileReader(path + "/config.properties");
        Properties p = new Properties();
        p.load(reader);
        List<String> propFiles = new ArrayList<>();
        String prefix;
        String type = "";

        Enumeration<Object> keys = p.keys();
        while (keys.hasMoreElements()) {
            Object key = keys.nextElement();
            if (key.toString().contains("robot.")) {
                propFiles.add(p.get(key).toString());
            }
            if (!key.toString().contains("robot.")) {
                properties.put(key.toString(), p.get(key).toString());
            }

        }


        for (String prop : propFiles) {
            reader = new FileReader(path + "/" + prop);
            p = new Properties();
            p.load(reader);
            keys = p.keys();
            while (keys.hasMoreElements()) {
                Object key = keys.nextElement();
                properties.put(key.toString(), p.get(key).toString());
            }
        }




        for (Map.Entry<String, String> entry : properties.entrySet()) {
            try {
                type = "";
                if (entry.getValue().contains("(")) {
                    type = entry.getValue().substring(entry.getValue().indexOf("(") + 1, entry.getValue().indexOf(")"));
                }
                if (entry.getKey().contains(".")) {
                    prefix = entry.getKey().substring(0, entry.getKey().indexOf("."));
                } else {
                    prefix = "general";
                }
                switch (type.toLowerCase()) {
                    case "int":
                        defaultVal = 0;
                        break;
                    case "number":
                        defaultVal = 0.0;
                        break;
                    case "boolean":
                        defaultVal = false;
                        break;
                    default:
                        defaultVal = "";
                        break;
                }

                if (allConstants.containsKey(prefix)) {
                    if (allConstants.get(prefix).containsKey(entry.getKey())) {
                        if (!allConstants.get(prefix).get(entry.getKey()
                                        .replace(prefix + ".", ""))
                                .equals(new propItem(quote(type), entry.getKey().replace(prefix + ".", ""),
                                        defaultVal.toString()))) {
                            throw new Exception("Build Failed: Error in Generating Constants");
                        }
                    } else {
                        allConstants.get(prefix).put(entry.getKey().replace(prefix + ".", ""), new propItem(type,
                                entry.getKey().replace(prefix + ".", ""),
                                defaultVal.toString()));
                    }
                } else {
                    allConstants.put(prefix, new HashMap<String, propItem>());
                    allConstants.get(prefix).put(entry.getKey().replace(prefix + ".", ""), new propItem(type,
                            entry.getKey().replace(prefix + ".", ""),
                            defaultVal.toString()));
                }


            } catch (Exception e) {
                e.printStackTrace();
            }
        }

    }


    public static void main(String[] args) throws Exception {

        init(args[0]);
        programWriter();

        try {
            // Creates a Writer using FileWriter
            FileWriter output = new FileWriter("src/main/java/frc/robot/NewConstants.java");

            // Writes the program to file
            output.write(program);
            System.out.println("Data is written to the file.");

            // Closes the writer
            output.close();
        } catch (Exception e) {
            e.getStackTrace();
        }

        for (Map.Entry<String, Map<String, propItem>> entry : allConstants.entrySet()) {
            System.out.println(entry);
        }
    }

    public static String program =
            "/* This file was autogenerated at {" + java.time.LocalTime.now().format(java.time.format.DateTimeFormatter.ofPattern("HH:mm:ss")) + "} - do not edit it - any edits will be overwritten! */ \n" +
                    "package frc.robot;\n" +
                    "\n" +
                    "import edu.wpi.first.math.geometry.*;\n" +
                    "import edu.wpi.first.math.util.Units;\n" +
                    "import frc.robot.RobotState;\n" +
                    "import frc.robot.subsystems.arm.Arm;\n" +
                    "import frc.utils.configfile.StormProp;\n" +
                    "public final class NewConstants {\n";


    public static void programWriter() {
        for (Map.Entry<String, Map<String, propItem>> entry : allConstants.entrySet()) {
            if (!entry.getKey().equals("general")) {
                program += "\npublic static final class " + entry.getKey().substring(0, 1).toUpperCase() + entry.getKey().substring(1)
                        + " {";
                for (Map.Entry entry2 : entry.getValue().entrySet()) {
                    program += entry2.getValue().toString();
                }

                program += "}";
            } else {
                for (Map.Entry entry2 : entry.getValue().entrySet()) {
                    program += "\t" + entry2.getValue().toString();
                }
            }
        }
        program += "}";
    }
}


