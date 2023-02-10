package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class DroidRageConstants {
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double STICK_DEADZONE = 0.2;
    }
    
    public static double getNumber(String name, double value) {
        double myValue = SmartDashboard.getNumber(name, value);
        if (myValue == value) {
            SmartDashboard.putNumber(name, value);
        }
        return myValue;
    }

    public static boolean getBoolean(String name, boolean value) {
        boolean myValue = SmartDashboard.getBoolean(name, value);
        if (myValue == value) {
            SmartDashboard.putBoolean(name, value);
        }
        return myValue;
    }

    public static String getString(String name, String value) {
        String myValue = SmartDashboard.getString(name, value);
        if (myValue == value) {
            SmartDashboard.putString(name, value);
        }
        return myValue;
    }
}
