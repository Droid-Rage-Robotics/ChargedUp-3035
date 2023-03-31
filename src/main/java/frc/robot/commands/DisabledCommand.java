package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DisabledCommand extends CommandBase {
    private final Command command;

    private DisabledCommand(Command command) {
        this.command = command;
    }

    public static DisabledCommand create(Command command) {
        return new DisabledCommand(command);
    }

    @Override
    public Set<Subsystem> getRequirements() { // getRequirements is run when the command is scheduled
        return command.getRequirements();
    }
    
    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.isFinished();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}