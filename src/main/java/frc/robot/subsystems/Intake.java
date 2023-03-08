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

public class Intake extends SubsystemBase {
    public enum IntakeSpeeds {
        FARCUBELOW(0),
        FARCUBEMID(0),
        FARCUBEHIGH(0),
        CONE(0),
        CONTINUOUS(1),
        INTAKE(0.3),
        OUTTAKE(-0.3),
        HOLDCONE(0.05),
        HOLDCUBE(0.025),
        STOP(0),

        ;

        private MutableDouble velocity;

        private IntakeSpeeds(double velocity) {
            this.velocity = new SimpleWidgetBuilder<Double>(velocity, IntakeSpeeds.class.getSimpleName()+"/"+name()+"/Velocity", Intake.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
        }
    }


    private final CANSparkMax clawMotor;
    private final PIDController intakeController;
    private final RelativeEncoder intakeEncoder;
    private final DoubleSolenoid intakeSolenoid;
    private final PneumaticHub pneumaticHub;
    
    private IntakeSpeeds targetVelocity;
    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>
        (true, "Is Enabled", Intake.class.getSimpleName())
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .buildMutableBoolean();
    private final WriteOnlyDouble targetVelocityWriter = new WriteOnlyDouble
        (0.0, "Target Intake Velocity", "Intake");

    private boolean isOpen = false;

    public Intake() {
        clawMotor = new CANSparkMax(19, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setInverted(false);
        intakeController = new PIDController(0.01,0,0);
        targetVelocity = IntakeSpeeds.CONTINUOUS;
        intakeEncoder = clawMotor.getEncoder();


        if (isEnabled.get()) {
            pneumaticHub = new PneumaticHub(10);
        } else {
            pneumaticHub = new PneumaticHub(); // Since this is the wrong port, it should effectively disable the subsystem
        }
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 10);

        close();
    }

    @Override
    public void periodic() {
        setIntakePower(intakeController.calculate(getIntakeVelocity()));
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
            // case CONE->intakeSpeed.get();
            // case CUBE->intakeSpeed.get();
            // case NONE->intakeSpeed.get();
        };
    }

    public CommandBase runOuttake() {
        // return setVelocity(IntakeSpeeds.OUTTAKE);
        return switch(TrackedElement.get()) {
            case CONE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
            case CUBE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
            case NONE->setTargetVelocity(IntakeSpeeds.OUTTAKE);
            // case CONE->intakeSpeed.get();
            // case CUBE->intakeSpeed.get();
            // case NONE->intakeSpeed.get();
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

    // private void setClawPower(double power) {
    //     if (!isEnabled.get()) return;
    //     clawMotor.set(power);
    // }

    // private CommandBase runSetPower(double power){
    //     return runOnce(() -> setClawPower(power));
    // }

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
            this.targetVelocity = velocity;
        });
    }

    public IntakeSpeeds getTargetVelocity() {
        return targetVelocity;
    }
    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }
      
    private void setIntakePower(double power) {
        clawMotor.set(power);
    }
    
}
