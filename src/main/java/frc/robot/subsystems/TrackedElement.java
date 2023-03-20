package frc.robot.subsystems;

import frc.robot.utilities.ShuffleboardValue;

public abstract class TrackedElement {
    public enum Element {
        CONE,
        CUBE
        ;
    }
    
    private static volatile Element element = Element.CONE;
    private static final ShuffleboardValue<String> ElementWriter = ShuffleboardValue.create(element.name(), "Tracked Element", "Misc").build();
    
    public static synchronized Element get() {
        return TrackedElement.element;
    }

    public static boolean isCone() {
        return element == Element.CONE;
    }

    public static synchronized void set(Element element) {
        ElementWriter.set(element.name());
        TrackedElement.element = element; // discord
    }
}
