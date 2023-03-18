package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TrackedElement.Element;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyDouble;

public class Intake2 extends SubsystemBase {
    public enum IntakeSpeeds {
        // MAX_RPM(5676),
        FARCUBELOW(4000),
        FARCUBEMID(5000),
        FARCUBEHIGH(5400),

        CONE(1000),
        CONTINUOUS(600),
        INTAKE(-3000),
        OUTTAKE(3000),
        HOLDCONE(500),
        HOLDCUBE(500),
        STOP(0), 
        POSITION_TOLERANCE(5),

        ;

        private final MutableDouble velocityRPM;

        private IntakeSpeeds(double velocityRPM) {
            this.velocityRPM = new SimpleWidgetBuilder<Double>(velocityRPM, IntakeSpeeds.class.getSimpleName()+"/"+name()+": Velocity (RPM)", Intake.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
        }
        public double get() {
            return velocityRPM.get();
        }
    }


    private final CANSparkMax intakeMotor;
    private final PIDController intakeController;
    private final RelativeEncoder intakeEncoder;
    private final DoubleSolenoid intakeSolenoid;
    private PneumaticHub pneumaticHub;
    
    // private IntakeSpeeds targetVelocity;
    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>
        (true, "Is Enabled", Intake.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .buildMutableBoolean();
    private final WriteOnlyDouble targetVelocityWriter = new WriteOnlyDouble
        (0, "Target Intake Velocity", "Intake");

    private boolean isOpen = false;

    public Intake2() {
        intakeMotor = new CANSparkMax(19, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);

        intakeController = new PIDController(
            0.000173611,
            0,
            0);
        // targetVelocity = IntakeSpeeds.CONTINUOUS;
        intakeEncoder = intakeMotor.getEncoder();
        intakeController.setTolerance(IntakeSpeeds.POSITION_TOLERANCE.get());


        pneumaticHub = new PneumaticHub(10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 11);

        close();
    }

    @Override
    public void periodic() {
        setIntakePower(intakeController.calculate(getIntakeVelocity()));
        if (!isEnabled.get()) {
            pneumaticHub.disableCompressor();
        }  
    }
  
    @Override
    public void simulationPeriodic() {}
  
    public void close() {
        if (!isEnabled.get()) return;
        intakeSolenoid.set(Value.kForward);//TODO:change
        isOpen = false;
        TrackedElement.set(Element.CONE); 
    }

    public void open() {
        if (!isEnabled.get()) return;
        intakeSolenoid.set(Value.kReverse);//TODO:change
        isOpen = true;
        TrackedElement.set(Element.CUBE);
    }
  
    public CommandBase toggleCommand() {
        return runOnce(() -> {
            if(isOpen) close();
            else open();
        });
    }

    public CommandBase runClose() {
        return runOnce(() -> close());//Cone
    }

    public CommandBase runOpen() {
        return runOnce(() -> open());//Cube
    }

    public CommandBase runIntake() {
        return switch(TrackedElement.get()) {
            case CONE->setTargetVelocity(IntakeSpeeds.INTAKE);
            case CUBE->setTargetVelocity(IntakeSpeeds.INTAKE);
            case NONE->setTargetVelocity(IntakeSpeeds.INTAKE);
        };
    }

    public CommandBase runOuttake() {
        return switch(TrackedElement.get()) {
            case CONE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
            case CUBE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
            case NONE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
        };
    }

    public CommandBase runStop() { 
        return setTargetVelocity(IntakeSpeeds.STOP);
    }

    public CommandBase runHoldIntake() { 
        return switch(TrackedElement.get()) {
            case CONE -> setTargetVelocity(IntakeSpeeds.HOLDCONE);
            case CUBE -> setTargetVelocity(IntakeSpeeds.HOLDCUBE);
            case NONE -> setTargetVelocity(IntakeSpeeds.HOLDCUBE);
            
        };
    }

    public CommandBase runIntakeFor(double wait) {
        return Commands.sequence(
            runIntake(),
            Commands.waitSeconds(wait),
            runStop()
        );
    }
    public CommandBase runOuttakeFor(double wait) {
        return Commands.sequence(
            runOuttake(),
            Commands.waitSeconds(wait),
            runStop()
        );
    }

    private CommandBase setTargetVelocity(IntakeSpeeds velocity) {
        return runOnce(() -> {
            intakeController.setSetpoint(velocity.get());
            targetVelocityWriter.set(velocity.get());
            // this.targetVelocity = velocity;
            
        });
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }
      
    private void setIntakePower(double power) {
        if (!isEnabled.get()) return;
        intakeMotor.set(power);
    }
    
    public CommandBase shootHighCube(){ 
        return setTargetVelocity(IntakeSpeeds.FARCUBEHIGH);
    }
    public CommandBase shootMidCube(){ 
        return setTargetVelocity(IntakeSpeeds.FARCUBEMID);
    }
    public CommandBase shootLowCube(){ 
        return setTargetVelocity(IntakeSpeeds.FARCUBELOW);
    }
}
