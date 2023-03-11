package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;

public class DropCone extends SequentialCommandGroup {
    public DropCone(Elevator elevator, Pivot pivot, Intake intake) {
        addCommands(
            elevator.dropVerticalElevator(),
            Commands.waitSeconds(0.2),
            intake.runOpen(),
            // intake.runOuttakeFor(1),
            elevator.moveInHorizontalElevator(),
            // Commands.waitSeconds(2),
            new MoveHold(elevator, pivot)
        );
    }
}
