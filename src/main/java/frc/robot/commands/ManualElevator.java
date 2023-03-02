package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
        elevator.setPosition(
            elevator.getTargetVerticalHeight()+(-operator.getRightY()*5), 
            elevator.getTargetHorizontalDistance()//+(operator.getRightX()*10)
            );
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

