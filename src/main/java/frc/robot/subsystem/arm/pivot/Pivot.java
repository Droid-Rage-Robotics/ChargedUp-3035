package frc.robot.subsystem.arm.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;

public class Pivot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 3 / 1;
        public static final double READINGS_PER_REVOLUTION = 1;
        public static final double ROTATIONS_TO_RADIANS = (GEAR_RATIO * READINGS_PER_REVOLUTION) / (Math.PI * 2);
    }

    protected final SafeCanSparkMax motor;
    protected final PIDController controller;
    protected ArmFeedforward feedforward;
    protected final RelativeEncoder encoder;

    protected final ShuffleboardValue<Double> encoderPositionWriter = ShuffleboardValue.create(0.0, "Encoder Position (Degrees)", Pivot.class.getSimpleName())
        .withSize(1, 3)
        .build();

    protected final ShuffleboardValue<Boolean> isMovingManually = ShuffleboardValue.create(false, "Moving manually", Pivot.class.getSimpleName())
        .build();
    
    public Pivot() {
        motor = new SafeCanSparkMax(
            18, 
            MotorType.kBrushless,
            ShuffleboardValue.create(false, "Is Enabled", Pivot.class.getSimpleName())
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .build(),
            ShuffleboardValue.create(0.0, "Power", Pivot.class.getSimpleName())
                .build()
        );
        motor.setIdleMode(IdleMode.kBrake);

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(Constants.ROTATIONS_TO_RADIANS);
  

        controller = new PIDController(0, 0, 0);//0.024
        controller.setTolerance(0.0);

        feedforward = new ArmFeedforward(0.01, 0, 0, 0);

        ComplexWidgetBuilder.create(controller, "PID Controller", Pivot.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 1);

        ComplexWidgetBuilder.create(runOnce(this::resetEncoder), "Reset encoder", Pivot.class.getSimpleName());
    }

    @Override
    public void periodic() {
        // motor.set(calculateFeedforward(getTargetPosition(), 0.) + calculatePID(getTargetPosition()));
        setVoltage(calculateFeedforward(getTargetPosition(), 0.03) + calculatePID(getTargetPosition()));
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public void setMovingManually(boolean value) {
        isMovingManually.set(value);
    }

    public boolean isMovingManually() {
        return isMovingManually.get();
    }


    public void setTargetPosition(double positionRadians) {
        controller.setSetpoint(positionRadians);
    }

    public double getTargetPosition() {
        return controller.getSetpoint();
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
        return controller.calculate(getPosition(), positionRadians);
    }

    protected double getPosition() {
        double position = encoder.getPosition();
        encoderPositionWriter.write(position);
        return position;
    }
}  