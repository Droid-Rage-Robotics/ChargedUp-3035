package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.arm.pivot.Pivot;

public class ManualPivot extends CommandBase {
    public static class Constants {    }

    private final Pivot pivot;
    private final Supplier<Double> pivotMove;
    private final Timer timer = new Timer();
    
    public ManualPivot(Supplier<Double> pivotMove, Pivot pivot) {
        this.pivot = pivot;
        this.pivotMove = pivotMove;
        timer.reset();

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
     }

    @Override
    public void execute() {
        if (timer.get() > 0.08) return;
        timer.reset();

        double move = pivotMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        if (move == 0) {
            pivot.setMovingManually(false);
            return;
            
        }
        pivot.setTargetPosition(pivot.getEncoderPosition() + move * 0.5);
        pivot.setMovingManually(true);
        
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

