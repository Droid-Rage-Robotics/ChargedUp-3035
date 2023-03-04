package frc.robot.commands.Manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.DroidRageConstants;
import frc.robot.subsystems.Pivot;

public class ManualPivot extends CommandBase {
    public static class Constants {    }

    private final Pivot pivot;
    // private final CommandXboxController operator;
    private final Supplier<Double> pivotMove;
    
    public ManualPivot(Supplier<Double> yPivotMove, Pivot pivot) {
        this.pivot = pivot;
        this.pivotMove = yPivotMove;

        addRequirements(pivot);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double y = pivotMove.get(); // reveresed intentnioally
        pivot.setCurrentPositionManually(y);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

