package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class DropCone extends SequentialCommandGroup {
    public DropCone(Elevator elevator, Pivot pivot, Intake intake) {
        addCommands(
            elevator.dropVerticalElevator(),
            intake.runOpen(),
            intake.runOuttakeFor(1),
            elevator.moveInHorizontalElevator(),
            new WaitCommand(1),
            pivot.moveHold()
        );
    }
}
