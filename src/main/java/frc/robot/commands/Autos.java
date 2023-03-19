package frc.robot.commands;

import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.LockWheels;
import frc.robot.commands.Drive.PathPlannerFollow;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
    /** Example static factory for an autonomous command. */

    // public static CommandBase top(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
    //     return PathPlannerFollow.create(drive, "Top")
    //         .setMaxVelocity(0.3)
    //         .setAcceleration(0.4)
    //         .addMarker("preloadDrop", new SequentialCommandGroup(
    //             new AutoMoveMid(elevator,pivot),
    //             new WaitCommand(1),
    //             new DropCone(elevator, pivot, intake)
    //             ))
    //         .addMarker("pickUp", new SequentialCommandGroup(
    //             new MoveIntakeLow(elevator, pivot),
    //             new IntakeCube(pivot, intake)
    //             ))
    //         .addMarker("dropOne", new SequentialCommandGroup(
    //             new MoveHigh(elevator,pivot),
    //             new WaitCommand(1),
    //             new DropCone(elevator, pivot, intake)
    //             ))
    //             // drive.resetHeading(270)
    //         .build();
    // }

    public static CommandBase dropPlusPark(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(0.1)
                ),
            PathPlannerFollow.create(drive, "Drop+Park")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }

    public static CommandBase bottom(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return PathPlannerFollow.create(drive, "Bottom")
            .setMaxVelocity(1.)
            .setAcceleration(1)
            .addMarker("preloadDrop", new SequentialCommandGroup(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(1),
                new DropCone(elevator, pivot, intake)
                ))
            .addMarker("pickUp", new SequentialCommandGroup(
                new MoveIntakeLow(elevator, pivot),
                new IntakeCube(pivot, intake,6)
                ))
            .build();
    }

    /*public static CommandBase strafeRight(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "StrafeBlue")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }*/
    public static CommandBase charge(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(1),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(0.1)
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
            
            drive.setOffsetCommand(180)
        );
    }
    // public static CommandBase charge2(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
    //     return Commands.sequence(
    //         Commands.sequence(
    //             new AutoMoveMid(elevator,pivot),
    //             new WaitCommand(3),
    //             new DropCone(elevator, pivot, intake),
    //             new WaitCommand(0.1)
    //             ),
    //         PathPlannerFollow.create(drive, "Charge2")
    //             .setMaxVelocity(1)
    //             .setAcceleration(0.9)
    //             .addMarker("wait", Commands.waitSeconds(1))
    //             .build(),
    //         //Command for autobalance
    //         new AutoBalance(drive),
    //         Commands.waitSeconds(0.5),
    //         new LockWheels(drive)
    //     );
    // }
    /*public static CommandBase intake(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                new AutoMoveMid(elevator,pivot),
                new WaitCommand(3),
                new DropCone(elevator, pivot, intake),
                new WaitCommand(3)
                ),
            PathPlannerFollow.create(drive, "Intake")
                .setMaxVelocity(1)
                .setAcceleration(3)
                .addMarker("intake", new SequentialCommandGroup(
                    new MoveIntakeLow(elevator, pivot),
                    new IntakeCube(pivot, intake)
                    ))
                .build()
        );
    }*/


    public static CommandBase oneToCubeAndToDrop(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            new AutoMoveMid(elevator,pivot),
            new WaitCommand(1),
            new DropCone(elevator, pivot, intake),
            new WaitCommand(0.3),
            PathPlannerFollow.create(drive, "ToCube1")
                .setMaxVelocity(1)
                .setAcceleration(0.8) 
                // .addMarker(J
                .build(),
            new MoveIntakeLow(elevator, pivot)
            //  PathPlannerFollow.create(drive, "ToCube1Forward")
            //     .setMaxVelocity(1)
            //     .setAcceleration(0.8) 
            //     .addMarker("intake",new SequentialCommandGroup(new IntakeCube(pivot, intake, 6))
            //      )
            //     .build()


                // new IntakeCube(pivot, intake, 5),
            // new MoveHold(elevator, pivot)
            // PathPlannerFollow.create(drive, "ToDrop1")
            //     .setMaxVelocity(1)
            //     .setAcceleration(1)
            //     .addMarker("pickUp", 
            //         new SequentialCommandGroup(
            //             // new MoveHigh(elevator, pivot)
            //         ))
            //     .build(),
            //     new DropCube(elevator, pivot, intake),
            // PathPlannerFollow.create(drive, "ToCube1")
            //     .setMaxVelocity(1)
            //     .setAcceleration(1)
            //     .build()
        );
    }

    public static CommandBase threeToCubeAndToDrop(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {//Bottom Red/Top Blue
        return new SequentialCommandGroup(
            new AutoMoveMid(elevator,pivot),
            new WaitCommand(0.6),
            new DropCone(elevator, pivot, intake),
            new WaitCommand(0.3),
            PathPlannerFollow.create(drive, "ToCube3")
                .setMaxVelocity(1)
                .setAcceleration(0.9) 
                // .addMarker(J
                .build(),
            new MoveIntakeLow(elevator, pivot),
            new WaitCommand(0.3),
             PathPlannerFollow.create(drive, "ToCube1Forward")
                .setMaxVelocity(0.8)
                .setAcceleration(0.8) 
                .addMarker("intake",new SequentialCommandGroup(new IntakeCube(pivot, intake, 6))
                 )
                .build()


                // new IntakeCube(pivot, intake, 5),
            // new MoveHold(elevator, pivot)
            // PathPlannerFollow.create(drive, "ToDrop1")
            //     .setMaxVelocity(1)
            //     .setAcceleration(1)
            //     .addMarker("pickUp", 
            //         new SequentialCommandGroup(
            //             // new MoveHigh(elevator, pivot)
            //         ))
            //     .build(),
            //     new DropCube(elevator, pivot, intake),
            // PathPlannerFollow.create(drive, "ToCube1")
            //     .setMaxVelocity(1)
            //     .setAcceleration(1)
            //     .build()
        );
    }

    public static CommandBase preloadPlusPark(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            new AutoMoveMid(elevator,pivot),
            new WaitCommand(0.3),
            new DropCone(elevator, pivot, intake),
            new WaitCommand(0.1),
            PathPlannerFollow.create(drive, "PreloadPlusPark")
                .setMaxVelocity(1)
                .setAcceleration(1) 
                .build()
        );
    }
    public static CommandBase testShoot(Drive drive, Elevator elevator, Pivot pivot, Intake intake) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            new DropCone(elevator, pivot, intake),//Shoot Cube High
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
    public static CommandBase straightTest(Drive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StraightTest")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }
    public static CommandBase straightTestBack(Drive drive) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "StraightTestBack")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }
    public static CommandBase turnTest(Drive drive) {
        return new SequentialCommandGroup(
            PathPlannerFollow.create(drive, "TurnTest")
                .setMaxVelocity(0.3)
                .setAcceleration(0.3)
                .build()
        );
    }


    private Autos() {}
}
