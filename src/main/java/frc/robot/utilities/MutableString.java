package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class MutableString {
    private final GenericEntry entry;
    private final String defaultValue;

    public MutableString(String defaultValue, String title, String tab) {
        this.defaultValue = defaultValue;
        entry = Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .getEntry();
    }

    public MutableString(GenericEntry entry, String defaultValue) {
        this.entry = entry;
        this.defaultValue = defaultValue;
    }

    public String get() {
        return entry.getString(defaultValue);
    }

    public void set(String value) {
        entry.setString(value);
    }
}