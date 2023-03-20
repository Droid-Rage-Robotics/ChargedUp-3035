package frc.robot.commands.Manual;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        double y = squareInput(pivotMove.get()); // reveresed intentnioally
        if (Math.abs(y) < DroidRageConstants.Gamepad.OPERATOR_STICK_DEADZONE) y = 0;
        pivot.setTargetPosition(pivot.getTargetPosition() + y * 0.5);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private double squareInput(double input) {
        return input * Math.abs(input);
    }
}

