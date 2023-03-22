package frc.robot.subsystem.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystem.TrackedElement;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.ShuffleboardValueEnum;

public class IntakeWithPIDTalon extends Intake {
    public enum Velocity implements ShuffleboardValueEnum<Double> {
        SHOOT_CUBE_LOW(500),
        SHOOT_CUBE_MID(500),
        SHOOT_CUBE_HIGH(500),

        SHOOT_CONE_LOW(500),
        SHOOT_CONE_MID(500),
        SHOOT_CONE_HIGH(500),

        // CONE(50),
        CONTINUOUS(50),
        INTAKE(1000),
        OUTTAKE(-1000),
        // INTAKE(1500),
        // OUTTAKE(-500),
        HOLD_CONE(100),
        HOLD_CUBE(100),
        STOP(0),


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
    public enum IntakePID implements ShuffleboardValueEnum<Double> {
        INTAKE_P(0.02),
        INTAKE_I(0),
        INTAKE_D(0),
        INTAKE_F(0);
        private final ShuffleboardValue<Double> velocityRPM;

        private IntakePID(double velocityRPM) {
            this.velocityRPM = ShuffleboardValue.create(velocityRPM, Velocity.class.getSimpleName()+"/"+name()+": Velocity (RPM)", Intake.class.getSimpleName())
                .withSize(1, 3)
                .build();
        }

        @Override
        public ShuffleboardValue<Double> getShuffleboardValue() {
            return velocityRPM;
        }

    }
    // protected final PIDController controller;
    // protected final SimpleMotorFeedforward feedforward;
    // protected final RelativeEncoder encoder;
    protected final TalonFX intakeMotor;
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    protected final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create(0.0, "Target Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityWriter = ShuffleboardValue.create(0.0, "Encoder Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityErrorWriter = ShuffleboardValue.create(0.0, "Encoder Velocity Error", Intake.class.getSimpleName()).build();

    public IntakeWithPIDTalon() {
        // super();
        // controller = new PIDController(
        //     0.002, 
        //     0,
        //     0);
        intakeMotor = new TalonFX(19);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);

        intakeMotor.config_kP(0, IntakePID.INTAKE_P.get());
        intakeMotor.config_kI(0, IntakePID.INTAKE_I.get());   
        intakeMotor.config_kD(0, IntakePID.INTAKE_D.get());  
        intakeMotor.config_kF(0, IntakePID.INTAKE_F.get());  


        // controller.setTolerance(5);
        // feedforward = new SimpleMotorFeedforward(0.2, 0.0022, 0);
        intakeMotor.setInverted(false);
        // ComplexWidgetBuilder.create(controller, "PID Controller", Intake.class.getSimpleName());
        // ComplexWidgetBuilder.create(runOnce(this::resetEncoder), "Reset Encoder", Intake.class.getSimpleName());
    }

    @Override
    public void periodic() {
        super.periodic();
        setTargetVelocity(Velocity.INTAKE);
        // setVoltage(calculatePID(getTargetVelocity()) + calculateFeedforward(getTargetVelocity()));
    }

    public double getTargetVelocity() {
        // return controller.getSetpoint();
        return intakeMotor.getActiveTrajectoryPosition();
    }

    public void setTargetVelocity(Velocity velocity) {
        intakeMotor.set(TalonFXControlMode.Velocity, velocity.get(), DemandType.ArbitraryFeedForward, 1);
        targetVelocityWriter.set(velocity.get());
    }

    public double getEncoderVelocity() {
        double velocity = intakeMotor.getSelectedSensorVelocity();
        encoderVelocityWriter.write(velocity);
        encoderVelocityErrorWriter.write(getTargetVelocity() - velocity);
        return velocity;
    }

    public double getEncoderVelocityError() {
        return encoderVelocityErrorWriter.get();
    }

    // protected double calculatePID(double targetVelocity) {
    //     return controller.calculate(getEncoderVelocity(), targetVelocity);
    // }

    // protected double calculateFeedforward(double targetVelocity) {
    //     return feedforward.calculate(targetVelocity);
    // }

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
        // encoder.setPosition(0);
        intakeMotor.setSelectedSensorPosition(0);
    }

    public double getVoltage() {
        return intakeMotor.getSupplyCurrent();
    }

    @Override
    public void stop() {
        setTargetVelocity(Velocity.STOP);
    }

    public void setManualOutput(double speed){
        intakeMotor.set(ControlMode.PercentOutput, speed);
        // motorBack.set(ControlMode.PercentOutput, backSpeed);
      }
    
}
