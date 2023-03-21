package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;

public class ManualHorizontalElevator extends CommandBase {
    private final HorizontalElevator horizontalElevator;
    private final Supplier<Double> horizontalElevatorMove;
    
    public ManualHorizontalElevator(Supplier<Double> horizontalElevatorMove, HorizontalElevator horizontalElevator) {
        this.horizontalElevator = horizontalElevator;
        this.horizontalElevatorMove = horizontalElevatorMove;
        
        addRequirements(horizontalElevator);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double target = -horizontalElevatorMove.get();
        target = DroidRageConstants.squareInput(target);
        target = DroidRageConstants.applyDeadBand(target);
        horizontalElevator.setTargetPosition(horizontalElevator.getTargetPosition() + target * 0.1);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
