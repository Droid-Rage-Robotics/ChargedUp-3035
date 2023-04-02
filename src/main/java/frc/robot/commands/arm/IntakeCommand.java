package frc.robot.commands.arm;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Light;
import frc.robot.utility.ShuffleboardValue;

public class IntakeCommand extends CommandBase {
    public enum IntakeState {
        INTAKE,
        ElEMENT_IN,
        TRACK_ELEMENT,
    }
    IntakeState intakeState = IntakeState.TRACK_ELEMENT;
    Timer intakeTimer = new Timer(); 
    Intake intake;
    Light light;
    CommandXboxController driver;


    public IntakeCommand(Intake intake, Light light, CommandXboxController driver) {
        this.intake = intake;
        this.light=light;
        this.driver=driver;
        addRequirements(light);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (intake.isElementIn() && driver.getRightTriggerAxis()>0.5 && intakeTimer.get() > 1){
            
            intakeState=IntakeState.ElEMENT_IN;
        } else if(driver.getRightTriggerAxis()>0.5){
            intakeState=IntakeState.INTAKE;
        }
        else intakeState=IntakeState.TRACK_ELEMENT;

        switch(intakeState){
            case ElEMENT_IN:
                // intake.runOnce(intake::intake);//Would you want to intake no matter what? 
                driver.getHID().setRumble(RumbleType.kBothRumble, 0.8);
                light.elementIn();
                break;
            case INTAKE:
            
                    intakeTimer.reset();intakeTimer.start();
                    intake.runOnce(intake::intake);
                // break;//Wanting for it to fall through
            case TRACK_ELEMENT:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                light.trackElementLight();
                break;
        }
        
        light.setIntakeState(intakeState);
    }

    @Override
    public void end(boolean interrupted) {
        intakeState=IntakeState.TRACK_ELEMENT;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
