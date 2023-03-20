package frc.robot.commands;

import frc.robot.commands.Drive.AutoBalance;
import frc.robot.commands.Drive.LockWheels;
import frc.robot.commands.Drive.PathPlannerFollow;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Arm.Position;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

    public static CommandBase dropPlusPark(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(3),
                new DropCone(arm, intake),
                Commands.waitSeconds(0.1)
                ),
            PathPlannerFollow.create(drive, "Drop+Park")
                .setMaxVelocity(1)
                .setAcceleration(1)
                .build()
        );
    }

    public static CommandBase bottom(Drive drive, Arm arm, Intake intake) {
        return PathPlannerFollow.create(drive, "Bottom")
            .setMaxVelocity(1.)
            .setAcceleration(1)
            .addMarker("preloadDrop", Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropCone(arm, intake)
            ))
            .addMarker("pickUp", new SequentialCommandGroup(
                arm.setPositionCommand(Position.INTAKE_LOW),
                new IntakeCube(arm, intake,6)
            ))
            .build();
    }

    public static CommandBase charge(Drive drive, Arm arm, Intake intake) {
        return Commands.sequence(
            Commands.sequence(
                arm.setPositionCommand(Position.AUTO_MID),
                Commands.waitSeconds(1),
                new DropCone(arm, intake),
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
            
            drive.setOffsetCommand(180)
        );
    }


    public static CommandBase oneToCubeAndToDrop(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return Commands.sequence(
            arm.setPositionCommand(Position.AUTO_MID),
            Commands.waitSeconds(1),
            new DropCone(arm, intake),
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
            new DropCone(arm, intake),
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
                .addMarker("intake",new SequentialCommandGroup(new IntakeCube(arm, intake, 6)))
                .build()

        );
    }

    public static CommandBase preloadPlusPark(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return Commands.sequence(
            arm.setPositionCommand(Position.AUTO_MID),
            Commands.waitSeconds(0.3),
            new DropCone(arm, intake),
            Commands.waitSeconds(0.1),
            PathPlannerFollow.create(drive, "PreloadPlusPark")
                .setMaxVelocity(1)
                .setAcceleration(1) 
                .build()
        );
    }
    public static CommandBase testShoot(Drive drive, Arm arm, Intake intake) {//Top Red/Bottom Blue
        return new SequentialCommandGroup(
            new DropCone(arm, intake),//Shoot Cube High
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
