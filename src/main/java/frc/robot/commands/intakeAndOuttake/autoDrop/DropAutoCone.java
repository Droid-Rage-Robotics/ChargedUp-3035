package frc.robot.commands.intakeAndOuttake.autoDrop;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class DropAutoCone extends SequentialCommandGroup {
    public DropAutoCone(Arm arm, Intake intake) {//TODO: MAKE IT DIFFERENT FOR SHOOT
        addCommands(
            intake.runOnce(()->intake.close(false)),
            // arm.lowerElevatorCommand(),
            intake.runFor(Velocity.SHOOT_CONE_HIGH, 1),
            Commands.waitSeconds(0.5),
            intake.runOnce(()->intake.open(true)),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}
