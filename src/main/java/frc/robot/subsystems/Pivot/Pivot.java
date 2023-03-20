package frc.robot.subsystems.Pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Position;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.SafeCanSparkMax;
import frc.robot.utilities.ShuffleboardValue;

public class Pivot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 3 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_RADIANS = (GEAR_RATIO * READINGS_PER_REVOLUTION) / (Math.PI * 2);
    }

    protected final SafeCanSparkMax motor;
    protected final PIDController pidController;
    protected ArmFeedforward feedforward;
    protected final RelativeEncoder encoder;

    protected final ShuffleboardValue<Double> encoderPositionWriter = ShuffleboardValue.create(0.0, "Encoder Position (Degrees)", Pivot.class.getSimpleName())
        .withSize(1, 3)
        .build();
    
    public Pivot() {
        motor = new SafeCanSparkMax(
            18, 
            MotorType.kBrushless,
            ShuffleboardValue.create(true, "Is Enabled", Pivot.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            ShuffleboardValue.create(0.0, "Power", Pivot.class.getSimpleName())
                .build()
        );
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(Constants.ROTATIONS_TO_RADIANS);
  

        pidController = new PIDController(0.024, 0, 0);//0.024
        pidController.setTolerance(0.10);

        feedforward = new ArmFeedforward(3, 0, 0, 0);

        ComplexWidgetBuilder.create(pidController, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 1);

        ComplexWidgetBuilder.create(runOnce(this::resetEncoder), "Reset encoder", Pivot.class.getSimpleName());
    }

    @Override
    public void periodic() {
        setVoltage(calculateFeedforward(getTargetPosition(), 0) + calculatePID(getTargetPosition()));
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public void setTargetPosition(double positionRadians) {
        pidController.setSetpoint(positionRadians);
    }

    public double getTargetPosition() {
        return pidController.getSetpoint();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    protected void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    protected double calculateFeedforward(double positionRadians, double velocity) {
        return feedforward.calculate(positionRadians, velocity);
    }

    protected double calculatePID(double positionRadians) {
        return pidController.calculate(getPosition(), positionRadians);
    }

    protected double getPosition() {
        double position = encoder.getPosition();
        encoderPositionWriter.write(position);
        return position;
    }
public CommandBase setPower(double power) {
    return runOnce(() -> {
        motor.set(power);
    });
}

}  