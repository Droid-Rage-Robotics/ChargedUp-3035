// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.IntakeAndOuttake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
public class DropCube extends SequentialCommandGroup {
    public DropCube(Arm arm, Intake intake) {
        switch(Arm.getPosition()){
            case AUTO_MID:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
                );
                break;
            case LOW:
                addCommands(
                        intake.runOnce(()->intake.close(false)),
                        intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_LOW))
                );
                break;
            case MID:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID))
                );
                break;
            case HIGH:
                addCommands(
                    intake.runOnce(()->intake.close(false)),
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE))
                );
                break;
            default:
                addCommands(
                    intake.runOnce(()->intake.outtake())
                );
                break;
            
        }
        // SuppliedCommand.create(() -> Commands.sequence(
        //     Commands.runOnce(() -> logPosition(targetPosition)),
        //     switch (targetPosition) {
        //         case AUTO_MID -> Commands.sequence(
        //             verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
        //             horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
        //             Commands.waitSeconds(1.4),
        //             pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
        //         ); 
                
        //         default -> Commands.sequence(
        //             verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
        //             horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
        //             pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
        //         );
        // )
        //     }}
    }
}
