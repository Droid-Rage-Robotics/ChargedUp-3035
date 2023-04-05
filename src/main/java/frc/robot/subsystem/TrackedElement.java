package frc.robot.subsystem;

import frc.robot.utility.ShuffleboardValue;

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

    public static boolean isCube() {
        return element == Element.CUBE;
    }

    public void toggleElement() {
        if(element==Element.CUBE) set(Element.CONE);
            else set(Element.CUBE);
    }
    

    public static synchronized void set(Element element) {
        
        TrackedElement.element = element;
        elementWriter.write(element.name());
        isCubeWriter.write(isCube());
    }
}
