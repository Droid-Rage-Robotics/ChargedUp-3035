package frc.robot.commands.arm;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Light;
import edu.wpi.first.wpilibj.Timer;

public class IntakeTeleopCommand extends SequentialCommandGroup {
    private enum IntakeState {
        INTAKE,
        ElEMENT_IN,
        TRACK_ELEMENT,
    }
    IntakeState intakeState = IntakeState.TRACK_ELEMENT;
    Timer intakeTimer = new Timer();
    public IntakeTeleopCommand(Intake intake, Light light, CommandXboxController driver) {
        intakeTimer.reset();
        if (intake.isElementIn() && driver.getRightTriggerAxis()>0.8 && intakeTimer.get() > 2){
            intakeState=IntakeState.ElEMENT_IN;
        }
        switch(intakeState){
            case ElEMENT_IN:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0.8);
                light.elementIn();
                break;
            case INTAKE:
                addCommands(
                    intake.runOnce(intake::intake)
                );
                break;
            case TRACK_ELEMENT:
                driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                light.trackElementLight();
                break;

        }
    }
}
