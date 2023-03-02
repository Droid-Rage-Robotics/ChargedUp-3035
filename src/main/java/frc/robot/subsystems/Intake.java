package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TrackedElement.Element;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;

public class Intake extends SubsystemBase {
    


    private final CANSparkMax clawMotor;
    private final DoubleSolenoid intakeSolenoid;
    private final PneumaticHub pneumaticHub;
    private final MutableBoolean isOpen = new MutableBoolean(false, "Is Open", Intake.class.getSimpleName());
    private final MutableDouble intakeSpeed = new MutableDouble(0.3, "Intake speed (power)", Intake.class.getSimpleName());
    private final MutableDouble outtakeSpeed = new MutableDouble(-0.3, "Outtake speed (power)", Intake.class.getSimpleName());

    public Intake() {
        clawMotor = new CANSparkMax(19, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setInverted(false);

        pneumaticHub = new PneumaticHub(10);
        // intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 10);
        intakeSolenoid = pneumaticHub.makeDoubleSolenoid(9, 10);
    }

    @Override
    public void periodic() {

    }
  
    @Override
    public void simulationPeriodic() {}
  
    public void open(){
        intakeSolenoid.set(Value.kForward);//TODO:change
        isOpen.set(true);
        TrackedElement.set(Element.CONE); 
    }

    public void close(){
        intakeSolenoid.set(Value.kReverse);//TODO:change
        isOpen.set(false);
        TrackedElement.set(Element.CUBE);
    }
  
    public CommandBase runToggleOpen() {
        return runOnce(() -> {
            if(isOpen.get()) close();
            else open();
        });
    }

    public CommandBase runOpen() {
        return runOnce(() -> open());
    }

    public CommandBase runClose() {
        return runOnce(() -> close());
    }

    public CommandBase runIntake() {
        return runSetPower(intakeSpeed.get());
    }

    public CommandBase runOuttake() {
        return runSetPower(outtakeSpeed.get());
    }

    public CommandBase runStop() { 
        return runSetPower(0);
    }

    private void setPower(double power) {
        clawMotor.set(power);
    }

    private CommandBase runSetPower(double power){
        return runOnce(() -> setPower(power));
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
            runIntake(),
            Commands.waitSeconds(wait),
            runStop()
        );
    }
}
