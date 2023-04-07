package frc.robot.commands.autoPaths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.drive.Drive;

public final class ChargeAutos {

    public static CommandBase chargeMid(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Just Charge")
                .setMaxVelocity(1.8)
                .setAcceleration(1.8)
                .addMarker("wait", Commands.waitSeconds(1))
                .addMarker("intake", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .addMarker("pickUp", new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                ))
                .build(),
                new AutoBalance(drive),
                new LockWheels(drive),
                drive.driveAutoReset(),

                arm.setPositionCommand(Position.MID),
                intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargeHigh(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            PathPlannerFollow.create(drive, "Just Charge")
                .setMaxVelocity(1.8)
                .setAcceleration(1.8)
                // .addMarker("wait", Commands.waitSeconds(1))
                .addMarker("intake", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .addMarker("pickUp", new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                ))
                .build(),
                new AutoBalance(drive),
                new LockWheels(drive),
                drive.driveAutoReset(),

                arm.setPositionCommand(Position.MID),
                intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargePlusPickUpMid(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge + PickupNormal")
                .setMaxVelocity(1.8)//Change to 1.8
                .setAcceleration(1.8)
                // .addMarker("wait", Commands.waitSeconds(1))
                .addMarker("down", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                    ))
                .addMarker("intake", new SequentialCommandGroup(
                    new WaitCommand(1),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.HOLD)
                )
            )
            .build(),
            new AutoBalance(drive),
            new LockWheels(drive),
            drive.driveAutoReset(),
            
            arm.setPositionCommand(Position.MID),
            intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargePlusPickUpHigh(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            PathPlannerFollow.create(drive, "Charge + PickupNormal")
                .setMaxVelocity(1.8)//Change to 1.8
                .setAcceleration(1.8)
                // .addMarker("wait", Commands.waitSeconds(1))
                .addMarker("down", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                    ))
                .addMarker("intake", new SequentialCommandGroup(
                    new WaitCommand(1),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.HOLD)
                ))
                .build(),
                new AutoBalance(drive),
                new LockWheels(drive),
                drive.driveAutoReset(),

                arm.setPositionCommand(Position.MID),
                intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargePlusPickUpPartsHigh(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            PathPlannerFollow.create(drive, "Charge + Pickup One")
                .setMaxVelocity(1.8)
                .setAcceleration(1.8)
                .addMarker("intake", new SequentialCommandGroup(
                    new WaitCommand(1),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.HOLD)
                )
            )
            .build(),
            
            //Try to use gyro to figure out when the bot is level to continue the path, or just go backwards to balance
            
            intake.runOnce(()->intake.open(true)),
            arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
            intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE)),

            PathPlannerFollow.create(drive, "Charge + Pickup Two")
                .setMaxVelocity(1.8)
                .setAcceleration(1.8)
                .addMarker("intake", new SequentialCommandGroup(
                    new WaitCommand(1),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.HOLD)
                )
            )
            .build(),

            new AutoBalance(drive),
            new LockWheels(drive),
            drive.driveAutoReset(),

            arm.setPositionCommand(Position.MID),
            intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    private ChargeAutos() {}
}
