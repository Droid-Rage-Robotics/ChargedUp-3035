package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

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
