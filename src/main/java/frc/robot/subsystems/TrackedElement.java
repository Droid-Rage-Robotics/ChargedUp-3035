package frc.robot.subsystems;

import frc.robot.DroidRageConstants;

public class TrackedElement {
    public enum Element {
        CONE,
        CUBE,
        NONE
        ;
    }
    
    private static volatile Element element;
    
    public static Element get() {
        return element;
    }

    public static void set(Element element) {
        DroidRageConstants.putString("Tracked Element", element.name());
        TrackedElement.element = element; // discord
    }
}
