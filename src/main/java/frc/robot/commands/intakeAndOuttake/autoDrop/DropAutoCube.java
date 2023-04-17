// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake.autoDrop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
public class DropAutoCube extends SuppliedCommand {
    public DropAutoCube(Arm arm, Intake intake) {
        super(()->switch(arm.getPosition()){
            case AUTO_MID-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
            case LOW-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_LOW)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
            case MID-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
            case HIGH-> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                );
            case AUTO_HIGH,AUTO_HOLD,AUTO_INTAKE_LOW,AUTO_LOW,HOLD,
                INTAKE_HIGH_DOUBLE_SUBSTATION,INTAKE_HIGH_SINGLE_SUBSTATION,
                INTAKE_LOW,INTAKE_LOW_DROPPED,START -> new SequentialCommandGroup(
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)),
                    new WaitCommand(0.2),
                    arm.setPositionCommand(Position.HOLD),
                    intake.runOnce(()->intake.stop())
                    //Doesn't hurt, since the only time
                    // outtake is used in high shot
                );
        });
    }
}