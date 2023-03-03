package frc.robot.commands.Manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends CommandBase {
    public static class Constants {    }

    private final Elevator elevator;
    private final CommandXboxController operator;
    
    public ManualElevator(CommandXboxController operator, Elevator elevator) {
        this.elevator = elevator;
        this.operator = operator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.isMovingManually = true;
     }

    @Override
    public void execute() {//TODO:Should we make it a different joystick? And move arm to 
        double y = -operator.getRightY();
        double x = operator.getRightX();
        if (Math.abs(operator.getLeftY()) > DroidRageConstants.Gamepad.STICK_DEADZONE) y = 0;
        if (Math.abs(operator.getLeftX()) > DroidRageConstants.Gamepad.STICK_DEADZONE) x = 0;
        
        elevator.setPosition(
            elevator.getTargetVerticalHeight()+(y*0.5), 
            elevator.getTargetHorizontalDistance()+(x*0.5)
            ).initialize();
        //TODO: Try to make it where it resets the value of the elevator
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        elevator.isMovingManually = false;
        return false;
    }
}

