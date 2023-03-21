package frc.robot.utility;

public abstract class StringStuff {
    public static String convert(String original) {
        original = original.replace("_", " ");
        original = original.toLowerCase();
        return original;
    }
}
