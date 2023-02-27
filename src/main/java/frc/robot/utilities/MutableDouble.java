package frc.robot.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class MutableDouble {
    private final GenericEntry entry;
    private final double defaultValue;

    public MutableDouble(double defaultValue, String title, String tab) {
        this.defaultValue = defaultValue;
        entry = Shuffleboard.getTab(tab)
            .addPersistent(title, defaultValue)
            .getEntry();
    }

    public MutableDouble(GenericEntry entry, double defaultValue) {
        this.entry = entry;
        this.defaultValue = defaultValue;
    }

    public double get() {
        return entry.getDouble(defaultValue);
    }

    public void set(double value) {
        entry.setDouble(value);
    }

    // protected void finalize() {
    //     entry.close();
    // }
}