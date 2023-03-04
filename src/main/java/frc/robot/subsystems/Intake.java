package frc.robot.subsystems;

import javax.sound.midi.Track;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

public class Intake extends SubsystemBase {
    


    private final CANSparkMax clawMotor;
    private final DoubleSolenoid intakeSolenoid;
    private final PneumaticHub pneumaticHub;
    // private final MutableBoolean isOpen = new MutableBoolean(false, "Is Open", Intake.class.getSimpleName());
    private final MutableDouble intakeSpeed = new MutableDouble(0.3, "Intake speed (power)", Intake.class.getSimpleName());
    private final MutableDouble outtakeSpeed = new MutableDouble(-0.3, "Outtake speed (power)", Intake.class.getSimpleName());

    private final MutableBoolean isEnabled = new SimpleWidgetBuilder<Boolean>(true, "Is Enabled", Intake.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

        private boolean isOpen = false;

    public Intake() {
        clawMotor = new CANSparkMax(19, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setInverted(false);

        if (isEnabled.get()) {
            pneumaticHub = new PneumaticHub(10);
        } else {
            pneumaticHub = new PneumaticHub(); // Since this is the wrong port, it should effectively disable the subsystem
        }
        

        // intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 10);

        close();
    }

    @Override
    public void periodic() {

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
  
    public CommandBase runToggleOpen() {
        return runOnce(() -> {
            if(!isOpen) open();//CUbe
            else close();//Cone
        });
    }

    public CommandBase runClose() {
        return runOnce(() -> close());//Cone
    }

    public CommandBase runOpen() {
        return runOnce(() -> open());//Cube
    }

    public CommandBase runIntake() {
        return runSetPower(switch(TrackedElement.get()) {
            case CONE->intakeSpeed.get();
            case CUBE->intakeSpeed.get();
            case NONE->intakeSpeed.get();
            
        });
    }

    public CommandBase runOuttake() {
        return runSetPower(outtakeSpeed.get());
    }

    public CommandBase runStop() { 
        return runSetPower(0);
    }

    public CommandBase runHoldIntake() { 
        return runSetPower(switch(TrackedElement.get()) {
            case CONE -> 0.05;
            case CUBE -> 0.025;
            case NONE -> 0.025;
            
        });
    }

    private void setClawPower(double power) {
        if (!isEnabled.get()) return;
        clawMotor.set(power);
    }

    private CommandBase runSetPower(double power){
        return runOnce(() -> setClawPower(power));
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
}
