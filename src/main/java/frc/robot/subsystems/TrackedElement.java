package frc.robot.subsystems;

import frc.robot.utilities.WriteOnlyString;

public class TrackedElement {
    public enum Element {
        CONE,
        CUBE,
        NONE
        ;
    }
    
    private static volatile Element element = Element.NONE;
    private static final WriteOnlyString ElementWriter = new WriteOnlyString(element.name(), "Tracked Element", "misc");
    
    public static Element get() {
        return element;
    }

    public static void set(Element element) {
        ElementWriter.set(element.name());
        TrackedElement.element = element; // discord
    }
}
