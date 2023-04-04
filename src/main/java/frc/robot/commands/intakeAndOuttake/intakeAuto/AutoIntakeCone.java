package frc.robot.commands.intakeAndOuttake.intakeAuto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class AutoIntakeCone extends SequentialCommandGroup {//TODO:FIx

    public AutoIntakeCone(Arm arm, Intake intake, int wait) {
        addCommands(
            intake.runOnce(()->intake.close(true)),
            intake.runIntakeFor(wait),
            Commands.waitSeconds(1),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}
