package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.intake.Intake;

public class DropCone extends SequentialCommandGroup {
    public DropCone(Arm arm, Intake intake) {
        addCommands(
            arm.lowerElevatorCommand(),
            Commands.waitSeconds(0.2),
            intake.runOnce(intake::open),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}
