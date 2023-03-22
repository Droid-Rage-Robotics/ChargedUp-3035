package frc.robot.subsystem.arm.elevator;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.SafeCanSparkMax;
import frc.robot.utility.ShuffleboardValue;

public abstract class Elevator extends SubsystemBase {
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
    protected abstract ShuffleboardValue<Boolean> getIsMovingManually();
    public abstract void resetEncoder();
    public abstract double getEncoderPosition();
    public abstract double getMaxPosition();
    public abstract double getMinPosition();
    

    public void setTargetPosition(double positionRadians) {
        if (positionRadians < getMinPosition()) return;
        if (positionRadians > getMaxPosition()) return;
        getController().setSetpoint(positionRadians);
    }

    public double getTargetPosition() {
        return getController().getSetpoint();
    }

    public void setMovingManually(boolean value) {
        getIsMovingManually().set(value);
    }

    public boolean isMovingManually() {
        return getIsMovingManually().get();
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