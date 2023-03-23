package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.arm.pivot.PivotMotionProfiled;

public class ManualMotionProfiledPivot extends CommandBase {
    public static class Constants {    }

    private final PivotMotionProfiled pivot;
    private final Supplier<Double> pivotMove;
    
    public ManualMotionProfiledPivot(Supplier<Double> pivotMove, PivotMotionProfiled pivot) {
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
        pivot.setTargetVelocity(move * 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setTarget(0, 0);
        pivot.setMovingManually(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

