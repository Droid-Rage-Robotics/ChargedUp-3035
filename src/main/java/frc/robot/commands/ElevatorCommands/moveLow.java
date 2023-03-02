package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class MoveLow extends SequentialCommandGroup {
    public MoveLow(Elevator elevator, Pivot pivot) {
        addCommands(
            elevator.moveLow(),
            pivot.moveLow()
        );
    }
}
