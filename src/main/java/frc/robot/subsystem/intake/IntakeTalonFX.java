package frc.robot.subsystem.intake;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.utility.SafeTalonFX;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.SafeMotor.IdleMode;

public class IntakeTalonFX extends IntakeWithPID {
    protected final SafeTalonFX motor = new SafeTalonFX(
        19,
        ShuffleboardValue.create(false, "Is Enabled", Intake.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            ShuffleboardValue.create(0.0, "Voltage", Intake.class.getSimpleName())
                .build()
    );

    public IntakeTalonFX() {
        motor.setIdleMode(IdleMode.Brake);
        motor.setInverted(false);
    }

    @Override
    public double getEncoderVelocity() {
        double velocity = encoder.getVelocity();
        encoderVelocityWriter.write(velocity);
        encoderVelocityErrorWriter.write(getTargetVelocity() - velocity);
        return velocity;
    }

    @Override
    protected void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
