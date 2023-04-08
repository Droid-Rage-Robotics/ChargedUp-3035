package frc.robot.commands.autoPaths;

import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.drive.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class BumpAutos {
    public static CommandBase onePlusOneBumpHigh_Mid(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),

            PathPlannerFollow.create(drive, "1+1 Bump")
                .setMaxVelocity(3)
                .setAcceleration(1)
                .addMarker("open", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true))
                ))
                .addMarker("intake", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .addMarker("pickUp", new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                ))
                .addMarker("shoot", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_MID),
                    new WaitCommand(0.8),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID)),
                    new WaitCommand(0.3),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
        );
    }

    public static CommandBase onePlusOneBumpMid_Mid(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.AUTO_MID),
            new WaitCommand(0.8),
            new DropAutoCone(arm, intake),

            PathPlannerFollow.create(drive, "1+1 Bump")
                .setMaxVelocity(3.5)
                .setAcceleration(1.5)
                .addMarker("open", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true)),
                    new WaitCommand(0.5),
                    arm.setPositionCommand(Position.HOLD)
                ))
                .addMarker("intake", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .addMarker("pickUp", new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                ))
                .addMarker("shoot", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_MID),
                    new WaitCommand(0.8),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID)),
                    new WaitCommand(0.3),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
        );
    }

    private BumpAutos() {}
}
