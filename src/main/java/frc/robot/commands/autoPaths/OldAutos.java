package frc.robot.commands.autoPaths;

import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.commands.intakeAndOuttake.autoIntake.AutoIntakeCube;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.drive.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class OldAutos {

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

    // public static CommandBase dropAndPickupContinnuous(Drive drive, Arm arm, Intake intake) {
    //     return Commands.sequence(
    //         arm.setPositionCommand(Position.HIGH),
    //         new WaitCommand(1.05),//HAS TO BE 1
    //         new DropAutoCone(arm, intake),
    //         // new WaitCommand(0.5),
    //         PathPlannerFollow.create(drive, "Drop+PickUp")
    //             .setMaxVelocity(4)
    //             .setAcceleration(2)
    //             .addMarker("intake2", new ParallelCommandGroup(
    //                 new AutoIntakeCube(arm, intake, 0))
    //             )
    //             .addMarker("pickUp", new SequentialCommandGroup(
    //                 // intake.runFor(Velocity.INTAKE,  2)
    //                 intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE)),
    //                 new WaitCommand(0.6),
    //                 intake.runOnce(intake::stop),
    //                 arm.setPositionCommand(Position.AUTO_MID)
    //                 // new WaitCommand(1),
    //                 // arm.setPositionCommand(Position.HOLD)
    //                 // intake.runOnce(()->intake.stop()^
    //                 )
    //             )
    //             .addMarker("shoot", new SequentialCommandGroup(
    //                 intake.runOnce(()-> intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID)),
    //                 // new WaitCommand(0.6),
    //                 intake.runOnce(intake::stop),
    //                 // intake.runOnce(()->intake.runFor(Velocity.SHOOT_AUTO_CUBE_MID, 2)), 
    //                 // new WaitCommand(1), 
    //                 arm.setPositionCommand(Position.HOLD))
    //             )
    //             .build(),
    //             drive.setOffsetCommand(drive.getRotation2d().rotateBy(Rotation2d.fromDegrees(0)).getDegrees())
    //     );
    // }

    private OldAutos() {}
}
