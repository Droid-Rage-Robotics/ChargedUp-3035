package frc.robot.subsystem.intake;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystem.TrackedElement;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.ShuffleboardValueEnum;

public class IntakeWithPID extends Intake {
    public enum Velocity implements ShuffleboardValueEnum<Double> {
        SHOOT_CUBE_LOW(500),
        SHOOT_CUBE_MID(500),
        SHOOT_CUBE_HIGH(500),

        SHOOT_CONE_LOW(500),
        SHOOT_CONE_MID(500),
        SHOOT_CONE_HIGH(500),

        // CONE(50),
        CONTINUOUS(50),
        INTAKE(1500),
        OUTTAKE(-500),
        HOLD_CONE(100),
        HOLD_CUBE(100),
        STOP(0)
        // SHOOT_CUBE_LOW(4000),
        // SHOOT_CUBE_MID(5000),
        // SHOOT_CUBE_HIGH(5400),

        // CONE(1000),
        // CONTINUOUS(600),
        // INTAKE(-3000),
        // OUTTAKE(3000),
        // HOLD_CONE(500),
        // HOLD_CUBE(500),
        // STOP(0)
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
    protected final PIDController controller;
    protected final SimpleMotorFeedforward feedforward;
    protected final RelativeEncoder encoder;
    protected final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create(0.0, "Target Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityWriter = ShuffleboardValue.create(0.0, "Encoder Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityErrorWriter = ShuffleboardValue.create(0.0, "Encoder Velocity Error", Intake.class.getSimpleName()).build();

    public IntakeWithPID() {
        super();
        controller = new PIDController(
            0.002, 
            0,
            0);
        encoder = intakeMotor.getEncoder();
        controller.setTolerance(5);
        feedforward = new SimpleMotorFeedforward(0.2, 0.0022, 0);
        intakeMotor.setInverted(false);
        ComplexWidgetBuilder.create(controller, "PID Controller", Intake.class.getSimpleName());
        ComplexWidgetBuilder.create(runOnce(this::resetEncoder), "Reset Encoder", Intake.class.getSimpleName());
    }

    @Override
    public void periodic() {
        super.periodic();
        setVoltage(calculatePID(getTargetVelocity()) + calculateFeedforward(getTargetVelocity()));
    }

    public double getTargetVelocity() {
        return controller.getSetpoint();
    }

    public void setTargetVelocity(Velocity velocity) {
        controller.setSetpoint(velocity.get());
        targetVelocityWriter.set(velocity.get());
    }

    public double getEncoderVelocity() {
        double velocity = encoder.getVelocity();
        encoderVelocityWriter.write(velocity);
        encoderVelocityErrorWriter.write(getTargetVelocity() - velocity);
        return velocity;
    }

    public double getEncoderVelocityError() {
        return encoderVelocityErrorWriter.get();
    }

    protected double calculatePID(double targetVelocity) {
        return controller.calculate(getEncoderVelocity(), targetVelocity);
    }

    protected double calculateFeedforward(double targetVelocity) {
        return feedforward.calculate(targetVelocity);
    }

    @Override
    public void intake() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> Velocity.INTAKE;
            case CUBE -> Velocity.INTAKE;
        });
    }

    @Override
    public void outtake() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> Velocity.OUTTAKE;
            case CUBE -> Velocity.OUTTAKE;
        });
    }

    @Override
    public void hold() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> Velocity.HOLD_CONE;
            case CUBE -> Velocity.HOLD_CUBE;
        });
    }

    public void shootHighCube() { 
        setTargetVelocity(Velocity.SHOOT_CUBE_HIGH);
    }
    public void shootMidCube() { 
        setTargetVelocity(Velocity.SHOOT_CUBE_MID);
    }
    public void shootLowCube() { 
        setTargetVelocity(Velocity.SHOOT_CUBE_LOW);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void stop() {
        setTargetVelocity(Velocity.STOP);
    }
}
