package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class WriteOnlyString {
    private final GenericEntry entry;

    public WriteOnlyString(String initialValue, String title, String tab) {
        entry = Shuffleboard.getTab(tab)
            .add(title, initialValue)
            .getEntry();
    }

    public WriteOnlyString(GenericEntry entry, String initialValue) {
        this.entry = entry;
    }

    public void set(String value) {
        entry.setString(value);
    }
}