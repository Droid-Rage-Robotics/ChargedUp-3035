package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.EnumPositions.TrackedElement;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.utilities.ShuffleboardValueEnum;

public class IntakeWithPID extends Intake {
    public enum Velocity implements ShuffleboardValueEnum<Double> {
        SHOOT_CUBE_LOW(4000),
        SHOOT_CUBE_MID(5000),
        SHOOT_CUBE_HIGH(5400),

        CONE(1000),
        CONTINUOUS(600),
        INTAKE(-3000),
        OUTTAKE(3000),
        HOLD_CONE(500),
        HOLD_CUBE(500),
        STOP(0)
        ;

        private final ShuffleboardValue<Double> velocityRPM;

        private Velocity(double velocityRPM) {
            this.velocityRPM = ShuffleboardValue.create(velocityRPM, Velocity.class.getSimpleName()+"/"+name()+": Velocity (RPM)", Intake.class.getSimpleName())
                .withSize(1, 3)
                .build();
        }

        @Override
        public ShuffleboardValue<Double> getShuffleboardValue() {
            return velocityRPM;
        }

    }
    protected final PIDController intakeController;
    protected final SimpleMotorFeedforward intakeFeedforward;
    protected final RelativeEncoder intakeEncoder;
    protected final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create(0.0, "Target Velocity", Intake.class.getSimpleName()).build();

    public IntakeWithPID() {
        super();
        intakeController = new PIDController(
            0.000173611, 
            0,
            0);
        intakeEncoder = intakeMotor.getEncoder();
        intakeController.setTolerance(5);
        intakeFeedforward = new SimpleMotorFeedforward(0.1, 0.0001761804, 0);

        ComplexWidgetBuilder.create(intakeController, "PID Controller", Intake.class.getSimpleName());
    }

    @Override
    public void periodic() {
        super.periodic();
        setVoltage(calculatePID(getTargetVelocity()) + calculateFeedforward(getTargetVelocity()));
    }

    protected double getTargetVelocity() {
        return intakeController.getSetpoint();
    }

    protected void setTargetVelocity(Velocity velocity) {
        intakeController.setSetpoint(velocity.get());
        targetVelocityWriter.set(velocity.get());
    }

    protected double getVelocity() {
        return intakeEncoder.getVelocity();
    }

    protected double calculatePID(double targetVelocity) {
        return intakeController.calculate(getVelocity(), targetVelocity);
    }

    protected double calculateFeedforward(double targetVelocity) {
        return intakeFeedforward.calculate(targetVelocity);
    }

    @Override
    public void intake() {
        switch(TrackedElement.get()) {
            case CONE -> setTargetVelocity(Velocity.INTAKE);
            case CUBE -> setTargetVelocity(Velocity.INTAKE);
        }
    }

    @Override
    public void outtake() {
        switch(TrackedElement.get()) {
            case CONE -> setTargetVelocity(Velocity.OUTTAKE);
            case CUBE -> setTargetVelocity(Velocity.OUTTAKE);
        }
    }

    @Override
    public void hold() {
        switch(TrackedElement.get()) {
            case CONE -> setTargetVelocity(Velocity.HOLD_CONE);
            case CUBE -> setTargetVelocity(Velocity.HOLD_CUBE);
        };
    }

    public void shootHighCube(){ 
        setTargetVelocity(Velocity.SHOOT_CUBE_HIGH);
    }
    public void shootMidCube(){ 
        setTargetVelocity(Velocity.SHOOT_CUBE_MID);
    }
    public void shootLowCube(){ 
        setTargetVelocity(Velocity.SHOOT_CUBE_LOW);
    }
}
