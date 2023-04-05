package frc.robot.commands.autoPaths;

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

    public static CommandBase charge(Drive drive, Arm arm, Intake intake) {
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



    public static CommandBase onePlusOneBump(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            new WaitCommand(0.5),
            arm.setPositionCommand(Position.HOLD),

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
                    arm.setPositionCommand(Position.HOLD)
                    )
                )
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
                intake.runOnce(()->intake.stop())    
        );
    }

    public static CommandBase onePlusOneFree(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),
            new WaitCommand(0.5),
            arm.setPositionCommand(Position.HOLD),

            PathPlannerFollow.create(drive, "1+1 Free")
                .setMaxVelocity(9)
                .setAcceleration(2)
                .addMarker("open", new SequentialCommandGroup(
                    intake.runOnce(()->intake.open(true))
                ))
                .addMarker("intake", new SequentialCommandGroup(//Make the robot go forward 
                    arm.setPositionCommand(Position.AUTO_INTAKE_LOW),
                    intake.runOnce(()-> intake.setTargetVelocity(Velocity.INTAKE))
                ))
                .addMarker("pickUp", new SequentialCommandGroup(
                    new WaitCommand(1),
                    intake.runOnce(intake::stop),
                    arm.setPositionCommand(Position.HOLD)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.HIGH),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    intake.runOnce(()->intake.hold()),
                    arm.setPositionCommand(Position.HOLD)
                    )
                )
                // .addMarker("shoot", new SequentialCommandGroup(
                //     arm.setPositionCommand(Position.AUTO_MID),
                //     new WaitCommand(0.8),
                //     intake.runOnce(()-> intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID)),
                //     new WaitCommand(0.3),
                //     intake.runOnce(intake::stop),
                //     arm.setPositionCommand(Position.INTAKE_LOW),
                //     intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                // ))
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop())
                
        );
    }

    private Autos() {}
}
