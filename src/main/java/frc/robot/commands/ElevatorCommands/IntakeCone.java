package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Arm.Position;

public class IntakeCone extends SequentialCommandGroup {
    public IntakeCone(Arm arm, Intake intake) {
        addCommands(
            intake.runOnce(intake::close),
            intake.runIntakeFor(3),
            Commands.waitSeconds(1),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}
