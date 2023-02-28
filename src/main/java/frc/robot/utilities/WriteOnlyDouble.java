package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class WriteOnlyDouble {
    private final GenericEntry entry;

    public WriteOnlyDouble(double initialValue, String title, String tab) {
        entry = Shuffleboard.getTab(tab)
            .add(title, initialValue)
            .getEntry();
    }

    public WriteOnlyDouble(GenericEntry entry, double defaultValue) {
        this.entry = entry;
    }

    public void set(double value) {
        entry.setDouble(value);
    }
}
