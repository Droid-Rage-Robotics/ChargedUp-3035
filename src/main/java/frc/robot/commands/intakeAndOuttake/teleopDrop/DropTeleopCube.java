// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake.teleopDrop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
public class DropTeleopCube extends SequentialCommandGroup {
    public DropTeleopCube(Arm arm, Intake intake) {
        switch(Arm.getPosition()){
            case AUTO_MID: // Should never be needed in Teleop
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
                );
                break;
            case LOW:
                addCommands(
                        intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_LOW))
                );
                break;
            case MID:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
                );
                break;
            case HIGH:
                addCommands(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE))
                );
                break;
            default:
                addCommands(
                    intake.runOnce(()->intake.outtake())
                );
                break;
            
        }
    }
}
