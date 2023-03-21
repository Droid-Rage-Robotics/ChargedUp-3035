package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.arm.pivot.Pivot;

public class ManualPivot extends CommandBase {
    public static class Constants {    }

    private final Pivot pivot;
    private final Supplier<Double> pivotMove;
    
    public ManualPivot(Supplier<Double> pivotMove, Pivot pivot) {
        this.pivot = pivot;
        this.pivotMove = pivotMove;

        addRequirements(pivot);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double target = pivotMove.get();
        target = DroidRageConstants.squareInput(target);
        target = DroidRageConstants.applyDeadBand(target);
        pivot.setTargetPosition(pivot.getTargetPosition() + target * 0.5);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

