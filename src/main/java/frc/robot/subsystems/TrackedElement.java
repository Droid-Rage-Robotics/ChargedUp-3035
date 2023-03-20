package frc.robot.subsystems;

import frc.robot.utilities.ShuffleboardValue;

public abstract class TrackedElement {
    public enum Element {
        CONE,
        CUBE
        ;
    }
    
    private static volatile Element element = Element.CONE;
    private static final ShuffleboardValue<String> elementWriter = ShuffleboardValue.create(element.name(), "Tracked Element", "Misc").build();
    private static final ShuffleboardValue<Boolean> isCubeWriter = ShuffleboardValue.create(isCube(), "Is Cube", "Misc")
        .withSize(4, 4)
        .build();
    
    public static synchronized Element get() {
        return TrackedElement.element;
    }

    private static boolean isCube() {
        return element == Element.CUBE;
    }

    public static synchronized void set(Element element) {
        elementWriter.write(element.name());
        isCubeWriter.write(isCube());
        TrackedElement.element = element;
    }
}
