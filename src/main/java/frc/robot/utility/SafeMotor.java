package frc.robot.utility;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class SafeMotor {
    protected final ShuffleboardValue<Boolean> isEnabled;
    protected final ShuffleboardValue<Double> outputWriter;
    public SafeMotor(ShuffleboardValue<Boolean> isEnabled, ShuffleboardValue<Double> outputWriter) {
        this.isEnabled = isEnabled;
        this.outputWriter = outputWriter;
    }

    public enum IdleMode {
        Coast,
        Brake
    }
    public abstract void setPower(double power);
    public abstract void setVoltage(double outputVolts);
    public abstract void setInverted(boolean isInverted);
    public abstract void setIdleMode(IdleMode mode);
}
