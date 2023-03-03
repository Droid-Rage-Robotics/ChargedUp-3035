package frc.robot.commands.Manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Pivot;

public class ManualPivot extends CommandBase {
    public static class Constants {    }

    private final Pivot arm;
    private final CommandXboxController operator;
    
    public ManualPivot(CommandXboxController operator, Pivot arm) {
        this.arm = arm;
        this.operator = operator;

        addRequirements(arm);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double y = operator.getLeftY();
        if (Math.abs(operator.getLeftY()) > DroidRageConstants.Gamepad.STICK_DEADZONE) y = 0;
        // arm.setTar(arm.getTargetPosition()+(y*1)).initialize();
        arm.setCurrentPositionManually(arm.getPosition()+y*1.5);
        //TODO: Try to make it where it resets the value of the elevator
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

