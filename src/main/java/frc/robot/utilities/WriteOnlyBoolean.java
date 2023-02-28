package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class WriteOnlyBoolean {
    private final GenericEntry entry;

    public WriteOnlyBoolean(boolean initialValue, String title, String tab) {
        entry = Shuffleboard.getTab(tab)
            .add(title, initialValue)
            .getEntry();
    }

    public WriteOnlyBoolean(GenericEntry entry, boolean initialValue) {
        this.entry = entry;
    }

    public void set(boolean value) {
        entry.setBoolean(value);
    }
}
