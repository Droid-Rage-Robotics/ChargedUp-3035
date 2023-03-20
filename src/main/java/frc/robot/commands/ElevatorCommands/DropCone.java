package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.Position;

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
