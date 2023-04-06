// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake.teleopDrop;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
public class DropTeleopCone extends SuppliedCommand {
    public DropTeleopCone(Arm arm, Intake intake) {
        super(()->switch(arm.getPosition()){
            // case AUTO_MID-> Commands.sequence(
            //         arm.lowerElevatorCommand(),
            //         Commands.waitSeconds(0.2),
            //         intake.runOnce(()->intake.open(true)),
            //         arm.setPositionCommand(Position.HOLD)
            //     );
            case LOW-> Commands.sequence(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_LOW))
                );
            case MID,AUTO_MID-> Commands.sequence(
                    arm.lowerElevatorCommand(),
                    Commands.waitSeconds(0.2),
                    intake.runOnce(()->intake.open(true)),
                    arm.setPositionCommand(Position.HOLD)
                );
            case HIGH-> Commands.sequence(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH))
                );
            default -> Commands.sequence(
                    // intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_AUTO_CUBE_MID))
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH))
                    //Doesn't hurt, since the only time
                    // outtake is used in high shot
                );
        });
    }
}
