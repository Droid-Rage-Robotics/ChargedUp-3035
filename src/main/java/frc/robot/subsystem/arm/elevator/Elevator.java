package frc.robot.subsystem.arm.elevator;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;

public abstract class Elevator extends SubsystemBase {
    protected final ShuffleboardValue<Double> encoderPositionWriter = ShuffleboardValue.create(0.0, "Encoder Position", getSimpleName())
        .withSize(1, 3)
        .build();

    protected final ShuffleboardValue<Boolean> isMovingManually = ShuffleboardValue.create(false, "Moving manually", getSimpleName())
        .build();

    public Elevator() {
        ComplexWidgetBuilder.create(getController(), " PID Controller", getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        ComplexWidgetBuilder.create(runOnce(this::resetEncoder), "Reset Elevator Encoders", getSimpleName());
    }

    @Override
    public void periodic() {
        setVoltage(calculatePID(getTargetPosition()) + calculateFeedforward(getTargetPosition()));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    protected abstract PIDController getController();
    protected abstract ElevatorFeedforward getFeedforward();
    protected abstract SafeCanSparkMax getMotor();
    protected abstract String getSimpleName();
    public abstract void resetEncoder();
    public abstract double getEncoderPosition();

    public void setTargetPosition(double positionRadians) {
        getController().setSetpoint(positionRadians);
    }

    public double getTargetPosition() {
        return getController().getSetpoint();
    }

    public void setMovingManually(boolean value) {
        isMovingManually.set(value);
    }

    public boolean isMovingManually() {
        return isMovingManually.get();
    }

    protected void setVoltage(double voltage) {
        getMotor().setVoltage(voltage);
    }

    protected double calculateFeedforward(double targetVelocity) {
        return getFeedforward().calculate(targetVelocity);
    }

    protected double calculatePID(double targetVelocity) {
        return getController().calculate(getEncoderPosition(), targetVelocity);
    }
}  