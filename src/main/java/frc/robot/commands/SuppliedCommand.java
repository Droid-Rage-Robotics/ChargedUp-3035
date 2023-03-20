package frc.robot.commands;

import java.util.Set;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class SuppliedCommand extends CommandBase {

    private final Supplier<Command> commandSupplier;
    private Command command = Commands.none();

    public SuppliedCommand(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public Set<Subsystem> getRequirements() { // getRequirements is run when the command is scheduled
        command = commandSupplier.get();
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
}
