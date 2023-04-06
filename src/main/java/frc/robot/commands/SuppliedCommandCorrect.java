package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

//Doesn't work - Secnd Practice Match: Reverted Changes
public class SuppliedCommandCorrect extends CommandBase {
    private final Supplier<Command> commandSupplier;
    private Command command = null;

    private SuppliedCommandCorrect(Supplier<Command> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    public static SuppliedCommand create(Supplier<Command> commandSupplier) {
        return new SuppliedCommand(commandSupplier);
    }
    
    @Override
    public void initialize() {
        command = commandSupplier.get();//TODO:Test
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            command.cancel();
        }
        command = null;
    }

    @Override
    public boolean isFinished() {
        return command == null || !command.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
