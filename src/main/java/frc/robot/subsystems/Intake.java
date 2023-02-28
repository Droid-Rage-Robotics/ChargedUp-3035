package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TrackedElement.Element;
import frc.robot.utilities.MutableBoolean;

public class Intake extends SubsystemBase {
    


    private final CANSparkMax clawMotor;
    private final DoubleSolenoid intakeSolenoid;
    private final MutableBoolean isOpen = new MutableBoolean(false, "Is Open", Intake.class.getSimpleName());
    private final double intakeSpeed = 0.3;
    private final double outtakeSpeed = -0.3;

    public Intake() {
        clawMotor = new CANSparkMax(19, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 10);
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
  
    public CommandBase toggle() {
        return runOnce(() -> {
            if(isOpen.get()) close();
            else if(!isOpen.get()) open();
        });
    }

    public CommandBase intake() {
        return runOnce(() -> {
            setPower(intakeSpeed); 
        });
        
    }

    public CommandBase outtake() {
        return runOnce(() -> setPower(outtakeSpeed));
    }

    public CommandBase stopIntake() { 
        return runOnce(() -> setPower(0));
    }

    private void setPower(double power){
        clawMotor.set(power);
    }    
}
