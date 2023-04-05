package frc.robot.commands.autoPaths;

import frc.robot.commands.drive.PathPlannerFollow;
import frc.robot.commands.intakeAndOuttake.autoDrop.DropAutoCone;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.drive.Drive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class TuningAutos {
    public static CommandBase forwardThenTurnTest(Drive drive) {
        return PathPlannerFollow.create(drive, "ForwardThenTurnTest")
            .setMaxVelocity(1)
            .setAcceleration(1)
            .build();
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


    private TuningAutos() {}
}
