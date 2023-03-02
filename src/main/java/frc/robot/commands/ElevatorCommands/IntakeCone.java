package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class IntakeCone extends SequentialCommandGroup {
    public IntakeCone( Pivot pivot, Intake intake) {
        addCommands(
            intake.runClose(),
            intake.runIntakeFor(3),
            Commands.waitSeconds(1),
            pivot.moveHold()
        );
    }
}
