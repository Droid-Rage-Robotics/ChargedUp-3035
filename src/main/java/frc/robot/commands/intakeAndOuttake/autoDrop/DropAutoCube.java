// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake.autoDrop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
public class DropAutoCube extends SequentialCommandGroup {
    public DropAutoCube(Arm arm, Intake intake) {
        switch(arm.getPosition()){
            case AUTO_MID: // Should never be needed in Teleop
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
                break;
            case LOW:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_LOW)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
                break;
            case MID:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
                break;
            case HIGH:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
                break;
            default:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                    //Doesn't hurt, since the only time
                    // outtake is used in high shot
                );
                break;
        }
    }
}