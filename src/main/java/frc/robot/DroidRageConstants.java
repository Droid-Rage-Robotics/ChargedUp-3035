package frc.robot;

public final class DroidRageConstants {
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DRIVER_STICK_DEADZONE = 0.05;//TODO::FIX
        public static final double OPERATOR_STICK_DEADZONE = 0.2;//TODO::FIX
    }

    public static double LOOP_TYPE_SECONDS = 0.02;
    public static double squareInput(double value) {
        return value * Math.abs(value);
    }
    public static double applyDeadBand(double value) {
        if (Math.abs(value) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE) value = 0;
        return value;
    }
}
