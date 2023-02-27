package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PIvot;

public class ManualArm extends CommandBase {
    public static class Constants {    }

    private final PIvot arm;
    private final CommandXboxController operator;
    
    public ManualArm(CommandXboxController operator, PIvot arm) {
        this.arm = arm;
        this.operator = operator;

        addRequirements(arm);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        arm.setArmPosition(arm.getTargetHeight()+(operator.getLeftY()*10));
        //TODO: Try to make it where it resets the value of the elevator
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

