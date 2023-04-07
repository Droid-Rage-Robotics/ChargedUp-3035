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
    public void initialize() {
        pivot.setMovingManually(true);
    }

    @Override
    public void execute() {
        double move = pivotMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        pivot.setTargetPosition(pivot.getTargetPosition() + move * 0.6);
        pivot.setMovingManually(true);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setTargetPosition(0);
        pivot.setMovingManually(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

