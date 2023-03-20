package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Position;
import frc.robot.subsystems.Intake.Intake;

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
