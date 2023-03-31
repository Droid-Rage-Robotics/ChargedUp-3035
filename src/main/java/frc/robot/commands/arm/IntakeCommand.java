package frc.robot.commands.arm;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Light;

public class IntakeCommand extends CommandBase {
    private enum IntakeState {
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
        intakeTimer.start();
    }

    @Override
    public void execute() {
        if (intake.isElementIn() && driver.getRightTriggerAxis()>0.8 && intakeTimer.get() > 2){
            intakeState=IntakeState.ElEMENT_IN;
        } else if(driver.getRightTriggerAxis()>0.6){
            intakeState=IntakeState.INTAKE;
        }
        else intakeState=IntakeState.TRACK_ELEMENT;

        switch(intakeState){
            case ElEMENT_IN:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0.8);
                light.elementIn();
                break;
            case INTAKE:
                    intakeTimer.reset();
                    intake.runOnce(intake::intake);
                break;
            case TRACK_ELEMENT:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                light.trackElementLight();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeState=IntakeState.TRACK_ELEMENT;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
