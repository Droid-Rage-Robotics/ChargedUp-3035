package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.arm.elevator.VerticalElevator;

public class ManualVerticalElevator extends CommandBase {
    private final VerticalElevator verticalElevator;
    private final Supplier<Double> verticalElevatorMove;
    
    public ManualVerticalElevator(Supplier<Double> verticalElevatorMove, VerticalElevator verticalElevator) {
        this.verticalElevator = verticalElevator;
        this.verticalElevatorMove = verticalElevatorMove;
        
        addRequirements(verticalElevator);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        double move = -verticalElevatorMove.get();
        move = DroidRageConstants.squareInput(move);
        move = DroidRageConstants.applyDeadBand(move);
        verticalElevator.setTargetPosition(verticalElevator.getTargetPosition() + move * 0.1);
        verticalElevator.setMovingManually(!(move == 0));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
