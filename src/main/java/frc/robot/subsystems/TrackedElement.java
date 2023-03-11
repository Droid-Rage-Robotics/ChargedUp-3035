package frc.robot.subsystems;

import java.util.function.Supplier;

import frc.robot.utilities.WriteOnlyString;

public class TrackedElement {
    public enum Element {
        CONE,
        CUBE,
        NONE
        ;
    }
    
    private static volatile Element element = Element.NONE;
    private static final WriteOnlyString ElementWriter = new WriteOnlyString(element.name(), "Tracked Element", "Misc");
    
    public static Element get() {
        return TrackedElement.element;
    }

    public static boolean isCone() {
        return element == Element.CONE;
    }

    public static void set(Element element) {
        ElementWriter.set(element.name());
        TrackedElement.element = element; // discord
    }
}
