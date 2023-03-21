package frc.robot.utility;

import com.revrobotics.CANSparkMax;

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
        outputWriter.write(speed);
        if (!isEnabled.get()) super.set(0);
            else super.set(speed);
    }

    @Override
    public void setVoltage(double outputVolts) {
        outputWriter.write(outputVolts);
        if (!isEnabled.get()) super.set(0);
            else super.setVoltage(outputVolts);
    }
}
