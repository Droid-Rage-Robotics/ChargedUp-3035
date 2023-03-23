package frc.robot.utility;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SafeCanSparkMax {
    private final ShuffleboardValue<Boolean> isEnabled;
    private final ShuffleboardValue<Double> outputWriter;
    private final CANSparkMax motor;
    public SafeCanSparkMax(int deviceId, MotorType type, ShuffleboardValue<Boolean> isEnabled, ShuffleboardValue<Double> outputWriter) {
        motor = new CANSparkMax(deviceId, type);
        this.isEnabled = isEnabled;
        this.outputWriter = outputWriter;
    }

    public void set(double speed) {
        outputWriter.write(speed);
        if (!isEnabled.get()) motor.set(0);
            else motor.set(speed);
    }

    public void setVoltage(double outputVolts) {
        outputWriter.write(outputVolts);
        if (!isEnabled.get()) motor.set(0);
            else motor.setVoltage(outputVolts);
    }

    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    public RelativeEncoder getEncoder() {
        return motor.getEncoder();
    }

    public void setIdleMode(IdleMode mode) {
        motor.setIdleMode(mode);
    }

    public SparkMaxAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
        return motor.getAbsoluteEncoder(encoderType);
    }

    private CANSparkMax getSparkMax() {
        return motor;
    }

    public void follow(SafeCanSparkMax leader, boolean invert) {
        motor.follow(leader.getSparkMax(), false);
    }

    public void burnFlash() {
        motor.burnFlash();
    }
}
