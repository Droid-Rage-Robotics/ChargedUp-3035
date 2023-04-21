package frc.robot.commands.autoPaths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.LockWheels;
import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Light;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.drive.Drive;

public final class ChargeAutos {
    public static CommandBase chargeMid(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Just Charge")
                .setMaxVelocity(1)
                .setAcceleration(1.)
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }

    public static CommandBase chargeHigh(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            PathPlannerFollow.create(drive, "Just Charge")
                .setMaxVelocity(1.)
                .setAcceleration(1.)
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }

    public static CommandBase chargePlusPickUpMid(Drive drive, Arm arm, Intake intake, Light light) {
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
            new AutoBalance(drive,light),
            new LockWheels(drive),
            drive.driveAutoReset(),
            
            arm.setPositionCommand(Position.MID),
            intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargePlusPickUpHigh(Drive drive, Arm arm, Intake intake, Light light) {
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
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset(),

                arm.setPositionCommand(Position.MID),
                intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
        );
    }

    public static CommandBase chargePlusTaxiMidCone(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            arm.setPositionCommand(Position.MID),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            PathPlannerFollow.create(drive, "Charge+Taxi")
                .setMaxVelocity(2.1)//Change to 1.8
                .setAcceleration(2)
                .addMarker("pushDown", new SequentialCommandGroup(
                    arm.pushChargeDownCommand()
                    )
                )
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }
    public static CommandBase chargePlusTaxiHighCube(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCube(arm, intake),
            PathPlannerFollow.create(drive, "Charge+Taxi180")
                .setMaxVelocity(1.8)//Change to 1.8
                .setAcceleration(1.8)
                .addMarker("pushDown", new SequentialCommandGroup(
                    arm.pushChargeDownCommand()
                    )
                )
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }

    public static CommandBase chargeMidTaxi180PickUp(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+Taxi180")
                .setMaxVelocity(3.5)
                .setAcceleration(1.5)
                .addMarker("pickUp", new SequentialCommandGroup(
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
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()//Don't Know If It Works
        );
    }

    public static CommandBase chargeMidTaxi180NoPickUp(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+Taxi180")
                .setMaxVelocity(3.5)
                .setAcceleration(1.5)
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()//Don't Know If It Works
        );
    }

    public static CommandBase chargeHighTaxi180NoPickUpCube(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_HIGH),
                Commands.waitSeconds(1),
                new DropAutoCube(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+Taxi180")
                .setMaxVelocity(3.5)
                .setAcceleration(1.5)
                .build(),
                new AutoBalance(drive, light),
                new LockWheels(drive),
                drive.driveAutoReset()//Don't Know If It Works
        );
    }

    public static CommandBase chargeMidTaxi90(Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+Taxi90")
                .setMaxVelocity(3)
                // .setAcceleration(1.)
                // .addMarker("pushDown", new SequentialCommandGroup(
                //     arm.lowerPivotCommand()
                //     )
                // )
                .build(),
                new AutoBalance(drive,light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }

    public static CommandBase chargeMidTaxi90Turn (Drive drive, Arm arm, Intake intake, Light light) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+Taxi90Turn")
                .setMaxVelocity(3)
                .setAcceleration(1.)
                .addMarker("pushDown", new SequentialCommandGroup(
                    arm.pushChargeDownCommand()
                    )
                )
                .build(),
                new AutoBalance(drive,light),
                new LockWheels(drive),
                drive.driveAutoReset()
        );
    }
    private ChargeAutos() {}
}
