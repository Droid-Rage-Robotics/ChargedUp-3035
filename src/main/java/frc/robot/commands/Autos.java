package frc.robot.commands;

import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.intakeAndOuttake.autoIntake.AutoIntakeCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.drive.Drive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

    // public static CommandBase dropPlusPark(Drive drive, Arm arm, Intake intake) {
    //     return Commands.sequence(
    //         Commands.sequence(
    //             arm.setPositionCommand(Position.AUTO_MID),
    //             Commands.waitSeconds(3),
    //             new DropCone(arm, intake),
    //             Commands.waitSeconds(0.1)
    //             ),
    //         PathPlannerFollow.create(drive, "Drop+Park")
    //             .setMaxVelocity(1)
    //             .setAcceleration(1)
    //             .build()
    //     );
    // }

    public static CommandBase forwardThenTurnTest(Drive drive) {
        return PathPlannerFollow.create(drive, "ForwardThenTurnTest")
            .setMaxVelocity(1)
            .setAcceleration(1)
            .build();
    }

    public static CommandBase bottom(Drive drive, Arm arm, Intake intake) {
        return PathPlannerFollow.create(drive, "Bottom")
            .setMaxVelocity(1.)
            .setAcceleration(1)
            .addMarker("preloadDrop", Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake)
            ))
            .addMarker("pickUp", new SequentialCommandGroup(
                arm.setPositionCommand(Position.INTAKE_LOW),
                new AutoIntakeCube(arm, intake,6)
            ))
            .build();
    }

    public static CommandBase charge(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge")
                .setMaxVelocity(1.8)
                .setAcceleration(1.8)
                // .addMarker("wait", Commands.waitSeconds(1))
                .build(),
            //Command for autobalance
            new AutoBalance(drive),
            // Commands.waitSeconds(0.5)^,
            // new LockWheels(drive),
            
            drive.setOffsetCommand(180)//Todo:Test
        );
    }

    public static CommandBase chargePlusPickUp(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropAutoCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Charge+PickUp")
                .setMaxVelocity(1.8)//Change to 1.8
                .setAcceleration(1.8)
                // .addMarker("wait", Commands.waitSeconds(1))
                .addMarker("down", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.INTAKE_LOW)
                    ))
                .addMarker("intake", new AutoIntakeCube(arm, intake, 7))
                .build(),
            //Command for autobalance
            new AutoBalance(drive),
            // Commands.waitSeconds(0.5)^,
            // new LockWheels(drive),
            drive.setOffsetCommand(180)//Todo:Test
        );
    }


    public static CommandBase oneToCubeAndToDrop(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return Commands.sequence(
            arm.setPositionCommand(Position.AUTO_MID),
            Commands.waitSeconds(1),
            new DropAutoCone(arm, intake),
            Commands.waitSeconds(0.3),
            PathPlannerFollow.create(drive, "ToCube1")
                .setMaxVelocity(1)
                .setAcceleration(0.8) 
                // .addMarker(J
                .build(),
            arm.setPositionCommand(Position.INTAKE_LOW)

        );
    }

    public static CommandBase threeToCubeAndToDrop(Drive drive, Arm arm, Intake intake) {//Bottom Red/Top Blue
        return Commands.sequence(
            arm.setPositionCommand(Position.AUTO_MID),
            Commands.waitSeconds(0.6),
            new DropAutoCone(arm, intake),
            Commands.waitSeconds(0.3),
            PathPlannerFollow.create(drive, "ToCube3")
                .setMaxVelocity(1)
                .setAcceleration(0.9) 
                // .addMarker(J
                .build(),
                arm.setPositionCommand(Position.INTAKE_LOW),
            Commands.waitSeconds(0.3),
             PathPlannerFollow.create(drive, "ToCube1Forward")
                .setMaxVelocity(0.8)
                .setAcceleration(0.8) 
                .addMarker("intake",new SequentialCommandGroup(new AutoIntakeCube(arm, intake, 6)))
                .build()

        );
    }

    public static CommandBase preloadPlusPark(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return Commands.sequence(
            arm.setPositionCommand(Position.AUTO_MID),
            Commands.waitSeconds(0.3),
            new DropAutoCone(arm, intake),
            Commands.waitSeconds(0.1),
            PathPlannerFollow.create(drive, "PreloadPlusPark")
                .setMaxVelocity(1)
                .setAcceleration(1) 
                .build()
        );
    }
    public static CommandBase testShoot(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            new DropAutoCone(arm, intake),//Shoot Cube High
            PathPlannerFollow.create(drive, "TestShoot")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .addMarker("intake", 
                    new SequentialCommandGroup(
                        // new MoveIntakeLow(elevator, pivot),
                        // new IntakeCube(pivot, intake, 4) //TODO:Test Wait Time
                    ))
                    .addMarker("pickUp", 
                        new SequentialCommandGroup(
                        // new MoveHigh(elevator, pivot)//SHoot Cube Mid or something
                    ))
                .build()
        );
    }

    public static CommandBase dropAndPickupContinnuous(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            // new WaitCommand(0.5),
            PathPlannerFollow.create(drive, "Drop+PickUp")
                .setMaxVelocity(4)
                .setAcceleration(2)
                .addMarker("intake2", new ParallelCommandGroup(
                    new AutoIntakeCube(arm, intake, 0))
                )
                .addMarker("pickUp", new SequentialCommandGroup(
                    // intake.runFor(Velocity.INTAKE,  2)
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE)),
                    new WaitCommand(0.6),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.AUTO_MID)
                    // new WaitCommand(1),
                    // arm.setPositionCommand(Position.HOLD)
                    // intake.runOnce(()->intake.stop()^
                    )
                )
                .addMarker("shoot", new SequentialCommandGroup(
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID)),
                    // new WaitCommand(0.6),
                    intake.runOnce(intake::stop),
                    // intake.runOnce(()->intake.runFor(Velocity.SHOOT_AUTO_CUBE_MID, 2)), 
                    // new WaitCommand(1), 
                    arm.setPositionCommand(Position.HOLD))
                )
                .build(),
                drive.setOffsetCommand(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(0)).getDegrees())
        );
    }



    //Auto Tests
    public static CommandBase forwardTest(Drive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "ForwardTest")
                .setMaxVelocity(1)
                .setAcceleration(3)
                .build()
        );
    }
    public static CommandBase backTest(Drive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "BackwardTest")
                .setMaxVelocity(1)
                .setAcceleration(3)
                .build()
        );
    }
    public static CommandBase turnTest(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TurnTest")
                .setMaxVelocity(0.2)
                .setAcceleration(0.2)
                .build(),
            drive.setOffsetCommand(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(180)).getDegrees())//Works
        );

    }
    public static CommandBase splineTest(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "SplineTest")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }
    public static CommandBase lineToLinearTest(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "LineToTest")
                .setMaxVelocity(0.2)
                .setAcceleration(0.2)
                .build()
        );
    }
    public static CommandBase strafeRight(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StrafeRightTest")
                .setMaxVelocity(0.2)
                .setAcceleration(0.2)
                .build()
        );
    }
    public static CommandBase strafeLeft(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StrafeLeftTest")
                .setMaxVelocity(0.2)
                .setAcceleration(0.2)
                .build()
        );
    }


    private Autos() {}
}
