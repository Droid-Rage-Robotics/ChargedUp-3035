package frc.robot.subsystem;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DisabledCommand;
import frc.robot.subsystem.TrackedElement.Element;
import frc.robot.subsystem.arm.Arm;
import frc.robot.utility.ComplexWidgetBuilder;
import frc.robot.utility.SafeTalonFX;
import frc.robot.utility.ShuffleboardValue;
import frc.robot.utility.ShuffleboardValueEnum;
import frc.robot.utility.SafeMotor.IdleMode;

public class Intake extends SubsystemBase {
    public enum Velocity implements ShuffleboardValueEnum<Double> {
        SHOOT_CUBE_LOW(3000),
        SHOOT_CUBE_MID(3000),
        // SHOOT_CUBE_HIGH(500),

        // SHOOT_CONE_LOW(500),
        // SHOOT_CONE_MID(500),
        SHOOT_CONE_HIGH(2800),

        //3000for pivot 140
        // CONE(50),
        CONTINUOUS(50),
        INTAKE(-3000),
        OUTTAKE(2000),//4500
        HOLD_CONE(100),
        HOLD_CUBE(100),
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

    protected final SafeTalonFX motor;
    protected final DoubleSolenoid intakeSolenoid;
    protected PneumaticHub pneumaticHub;
    protected static final boolean isOpenDefault = TrackedElement.get() == Element.CUBE ? true : false;
    protected final ShuffleboardValue<Boolean> isOpen = ShuffleboardValue.create(isOpenDefault, "Is Open", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Boolean> compressorEnabledWriter = ShuffleboardValue.create(true, "Compressor Enabled", Intake.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withSize(1, 2)
        .build();
    protected final ShuffleboardValue<Double> targetVelocityWriter = ShuffleboardValue.create(0.0, "Target Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityWriter = ShuffleboardValue.create(0.0, "Encoder Velocity", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> encoderVelocityErrorWriter = ShuffleboardValue.create(0.0, "Encoder Velocity Error", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> forwardPressureWriter = ShuffleboardValue.create(0.0, "Forward Pressure", Intake.class.getSimpleName()).build();
    protected final ShuffleboardValue<Double> backwardPressureWriter = ShuffleboardValue.create(0.0, "Backward Pressure", Intake.class.getSimpleName()).build();
    protected final PIDController controller;
    protected final SimpleMotorFeedforward feedforward;
        

    public Intake() {
        motor = new SafeTalonFX(
            19,
            ShuffleboardValue.create(true, "Is Enabled", Intake.class.getSimpleName())
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .build(),
                ShuffleboardValue.create(0.0, "Voltage", Intake.class.getSimpleName())
                    .build()
        );

        motor.setIdleMode(IdleMode.Brake);
        motor.setInverted(true);

        pneumaticHub = new PneumaticHub(10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 11);

        
        controller = new PIDController(
            0.0003,//0.0003 
            0,
            0);
        controller.setTolerance(5);
        feedforward = new SimpleMotorFeedforward(0.64, 0.000515, 0);
        ComplexWidgetBuilder.create(controller, "PID Controller", Intake.class.getSimpleName());
        ComplexWidgetBuilder.create(DisabledCommand.create(runOnce(this::resetEncoder)), "Reset Encoder", Intake.class.getSimpleName());
        close();
    }

    @Override
    public void periodic() {
        if (!compressorEnabledWriter.get()) pneumaticHub.disableCompressor();
        setVoltage(calculatePID(getTargetVelocity()) + calculateFeedforward(getTargetVelocity()));
        // forwardPressureWriter.set(pneumaticHub.getPressure(9));//TODO:Test:
        // backwardPressureWriter.set(pneumaticHub.getPressure(11));
    }
  
    public void close() {
        intakeSolenoid.set(Value.kForward);//TODO:change
        isOpen.set(false);
        TrackedElement.set(Element.CONE); 
    }

    public void open() {
        intakeSolenoid.set(Value.kReverse);//TODO:change
        isOpen.set(true);
        TrackedElement.set(Element.CUBE);
    }

    public void toggle() {
        if(isOpen.get()) close();
            else open();
    }

    public double getEncoderVelocity() {
        double velocity = motor.getVelocity();
        encoderVelocityWriter.write(velocity);
        encoderVelocityErrorWriter.write(getTargetVelocity() - velocity);
        return velocity;
    }

    public double getTargetVelocity() {
        return controller.getSetpoint();
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

    protected void setPower(double power) {
        motor.setPower(power);
    }

    protected void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void setTargetVelocity(Velocity velocity) {
        controller.setSetpoint(velocity.get());
        targetVelocityWriter.set(velocity.get());
    }

    public void intake() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> switch(Arm.getPosition()){
                case LOW ->Velocity.INTAKE;
                case MID ->Velocity.INTAKE;
                case HIGH ->Velocity.INTAKE;
                default -> Velocity.INTAKE;
            };
            case CUBE -> switch(Arm.getPosition()){
                case LOW ->Velocity.INTAKE;
                case MID ->Velocity.INTAKE;
                case HIGH ->Velocity.INTAKE;
                default -> Velocity.INTAKE;
            };
        });
    }

    public void outtake() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> switch(Arm.getPosition()){
                case LOW ->Velocity.OUTTAKE;
                case MID ->Velocity.OUTTAKE;
                case HIGH ->Velocity.SHOOT_CONE_HIGH;
                default -> Velocity.OUTTAKE;
            };
            case CUBE -> switch(Arm.getPosition()){
                case LOW ->Velocity.SHOOT_CUBE_LOW;
                case MID ->Velocity.SHOOT_CUBE_MID;
                case HIGH ->Velocity.OUTTAKE;
                default -> Velocity.OUTTAKE;
            };
        });
    }

    public void hold() {
        setTargetVelocity(switch(TrackedElement.get()) {
            case CONE -> Velocity.HOLD_CONE;
            case CUBE -> Velocity.HOLD_CUBE;
        });
    }

    // public void shootHighCube() { 
    //     setTargetVelocity(Velocity.SHOOT_CUBE_HIGH);
    // }
    
    // public void shootMidCube() { 
    //     setTargetVelocity(Velocity.SHOOT_CUBE_MID);
    // }

    // public void shootLowCube() { 
    //     setTargetVelocity(Velocity.SHOOT_CUBE_LOW);
    // }

    public void resetEncoder() {
        motor.setPosition(0);
    }

    public void stop() {
        setTargetVelocity(Velocity.STOP);
    }

    public CommandBase runIntakeFor(double waitSeconds) {
        return Commands.sequence(
            runOnce(this::intake),
            Commands.waitSeconds(waitSeconds),
            runOnce(this::stop)
        );
    }
    public CommandBase runOuttakeFor(double waitSeconds) {
        return Commands.sequence(
            runOnce(this::outtake),
            Commands.waitSeconds(waitSeconds),
            runOnce(this::stop)
        );
    }

    public boolean isElementIn(){
        return getEncoderVelocityError()<-2500;
    }
}

