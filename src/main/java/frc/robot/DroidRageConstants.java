package frc.robot;

public final class DroidRageConstants {
    public static class Gamepad {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final double DRIVER_STICK_DEADZONE = 0.05;//TODO::FIX
        public static final double OPERATOR_STICK_DEADZONE = 0.2;//TODO::FIX
    }

    // public static void registerSendable(Sendable sendable, String title, String tab) {
    //     Shuffleboard.getTab(tab)
    //         .add(title, sendable);
    // }

    // public static void registerVideo(VideoSource video, String title, String tab) {
    //     Shuffleboard.getTab(tab)
    //         .add(title, video);
    // }

    // public static SuppliedValueWidget<Double> makeShuffleboardDouble(DoubleSupplier valueSupplier, String title, String tab) {
    //     return Shuffleboard.getTab(tab)
    //         .addDouble(title, valueSupplier);
    // }
    
    // public static double getNumber(String name, double value) {
    //     double myValue = SmartDashboard.getNumber(name, value);
    //     // if (myValue == value) {
    //     //     SmartDashboard.putNumber(name, value);
    //     // }
    //     return myValue;
    // }

    // public static boolean getBoolean(String name, boolean value) {
    //     boolean myValue = SmartDashboard.getBoolean(name, value);
    //     if (myValue == value) {
    //         SmartDashboard.putBoolean(name, value);
    //     }
    //     return myValue;
    // }

    // public static String getString(String name, String value) {
    //     String myValue = SmartDashboard.getString(name, value);
    //     if (myValue == value) {
    //         SmartDashboard.putString(name, value);
    //     }
    //     return myValue;
    // }

    // public static void putNumber(String name, double value) {
    //     SmartDashboard.putNumber(name, value);
    // }

    // public static void putBoolean(String name, boolean value) {
    //     SmartDashboard.putBoolean(name, value);
    // }

    // public static void putString(String name, String value) {
    //     SmartDashboard.putString(name, value);
    // }
}
