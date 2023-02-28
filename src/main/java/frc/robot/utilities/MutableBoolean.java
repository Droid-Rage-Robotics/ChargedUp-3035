package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class MutableBoolean {
    private final GenericEntry entry;
    private final boolean value;

    public MutableBoolean(boolean defaultValue, String title, String tab) {
        this.value = defaultValue;
        entry = Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .getEntry();
    }

    public MutableBoolean(GenericEntry entry, boolean defaultValue) {
        this.entry = entry;
        this.value = defaultValue;
    }

    public boolean get() {
        return entry.getBoolean(value);
    }

    public void set(boolean value) {
        entry.setBoolean(value);
    }

    // protected void finalize() {
    //     entry.close();
    // }
}