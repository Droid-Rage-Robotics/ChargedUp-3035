package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.utilities.ShuffleboardValue;

public class SafeCanSparkMax extends CANSparkMax {
    private final ShuffleboardValue<Boolean> isEnabled;
    private final ShuffleboardValue<Double> outputWriter;
    public SafeCanSparkMax(int deviceId, MotorType type, ShuffleboardValue<Boolean> isEnabled, ShuffleboardValue<Double> outputWriter) {
        super(deviceId, type);
        this.isEnabled = isEnabled;
        this.outputWriter = outputWriter;
    }

    @Override
    public void set(double speed) {
        if (!isEnabled.get()) return;
        outputWriter.write(speed);
        super.set(speed);
    }

    @Override
    public void setVoltage(double outputVolts) {
        if (!isEnabled.get()) return;
        outputWriter.write(super.get());
        super.setVoltage(outputVolts);
    }
}
