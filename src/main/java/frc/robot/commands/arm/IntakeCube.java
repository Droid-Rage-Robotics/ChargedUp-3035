package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class IntakeCube extends SequentialCommandGroup {
    //TODO: Maybe make a position where the pivot iks downwards so cube doesn't roll away
    public IntakeCube(Arm arm, Intake intake, double wait) {
        addCommands(
            intake.runOnce(intake::open),
            intake.runIntakeFor(wait),
            arm.setPositionCommand(Position.HOLD)
        );
    }
    public IntakeCube(Arm arm, Intake intake) {
        this(arm, intake, 6.0);
    }
}
