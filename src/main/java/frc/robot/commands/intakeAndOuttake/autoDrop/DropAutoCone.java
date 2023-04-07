package frc.robot.commands.intakeAndOuttake.autoDrop;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;

public class DropAutoCone extends SuppliedCommand {
    public DropAutoCone(Arm arm, Intake intake) {//TODO: MAKE IT DIFFERENT FOR SHOOT
        super(()->
        switch(arm.getPosition()){
            case AUTO_MID-> new SequentialCommandGroup(
                    // arm.lowerElevatorCommand(),
                    Commands.waitSeconds(0.5),
                    intake.runOnce(()->intake.open(true)),
                    Commands.waitSeconds(0.3),
                    arm.setPositionCommand(Position.HOLD)
                );
            case LOW-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_LOW))
                );
            case MID-> new SequentialCommandGroup(
                    arm.lowerElevatorCommand(),
                    Commands.waitSeconds(0.2),
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.HOLD)
                );
            case HIGH-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)),
                    Commands.waitSeconds(0.5),
                    arm.setPositionCommand(Position.HOLD)
                );
            default-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH))
                    // Doesn't hurt, since the only time
                    // outtake is used in high shot
                );
    });
}
}