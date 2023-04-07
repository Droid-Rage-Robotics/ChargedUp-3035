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

public final class FreeAutos {
    public static CommandBase onePlusOneFreeHigh_High(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),

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
                    arm.setPositionCommand(Position.AUTO_HOLD),
                    intake.runOnce(intake::stop)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.HIGH),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                    )
                )
                .addMarker("intake2", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                    )
                )
                .build(),
                new WaitCommand(1.4),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
                
        );
    }

    public static CommandBase onePlusOneFreeMid_Mid(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.AUTO_MID),
            new DropAutoCone(arm, intake),

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
                    arm.setPositionCommand(Position.AUTO_HOLD),
                    intake.runOnce(intake::stop)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_MID),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                    )
                )
                .addMarker("intake2", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                    )
                )
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
                
        );
    }

    public static CommandBase onePlusOneFreeMid_MidPear(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.AUTO_MID),
            new DropAutoCone(arm, intake),

            PathPlannerFollow.create(drive, "1+1 Free Pear")
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
                    arm.setPositionCommand(Position.AUTO_HOLD),
                    intake.runOnce(intake::stop)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_MID),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                    )
                )
                .addMarker("intake2", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                    )
                )
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
                
        );
    }

    public static CommandBase onePlusOneFreeHigh_Mid(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.HIGH),
            new WaitCommand(1.05),//HAS TO BE 1
            new DropAutoCone(arm, intake),

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
                    arm.setPositionCommand(Position.AUTO_HOLD),
                    intake.runOnce(intake::hold)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_MID),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                    )
                )
                .addMarker("intake2", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                    )
                )
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.stop()),
                drive.driveAutoReset()
                
        );
    }

    public static CommandBase onePlusOneFreeMid_High(Drive drive, Arm arm, Intake intake) {
        return new SequentialCommandGroup(
            arm.setPositionCommand(Position.AUTO_MID),
            new DropAutoCone(arm, intake),

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
                    arm.setPositionCommand(Position.AUTO_HOLD),
                    intake.runOnce(intake::stop)
                    )
                )
                .addMarker("drop", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.AUTO_HIGH),
                    new WaitCommand(1.),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(1),
                    arm.setPositionCommand(Position.AUTO_HOLD)
                    )
                )
                .addMarker("intake2", new SequentialCommandGroup(
                    arm.setPositionCommand(Position.INTAKE_LOW),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.INTAKE))
                    )
                )
                .build(),
                new WaitCommand(1),
                intake.runOnce(()->intake.hold()),
                drive.driveAutoReset()
                
        );
    }

    private FreeAutos() {}
}
