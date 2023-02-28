package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class MutableInteger {
    private final GenericEntry entry;
    private final long defaultValue;

    public MutableInteger(long defaultValue, String title, String tab) {
        this.defaultValue = defaultValue;
        entry = Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .getEntry();
    }

    public MutableInteger(GenericEntry entry, long defaultValue) {
        this.entry = entry;
        this.defaultValue = defaultValue;
    }

    public long get() {
        return entry.getInteger(defaultValue);
    }

    public void set(long value) {
        entry.setDouble(value);
    }
}