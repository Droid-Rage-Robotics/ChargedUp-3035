// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.IntakeAndOuttake.teleopDrop;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
public class DropTeleopCone extends SequentialCommandGroup {
    public DropTeleopCone(Arm arm, Intake intake) {
        switch(Arm.getPosition()){
            case AUTO_MID: // Should never be needed in Teleop
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    new WaitCommand(0.5),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE))
                );
                break;
            case LOW:
                addCommands(
                        intake.runOnce(()->intake.close(false)),
                        new WaitCommand(0.5),
                        intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_LOW))
                );
                break;
            case MID:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    new WaitCommand(0.5),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE))
                );
                break;
            case HIGH:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    new WaitCommand(0.5),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH))
                );
                break;
            default:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    arm.lowerElevatorCommand(),
                    Commands.waitSeconds(0.2),
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.HOLD)
                );
                break;
            
        }
    }
}
