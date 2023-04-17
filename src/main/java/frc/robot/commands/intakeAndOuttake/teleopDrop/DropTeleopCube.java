// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake.teleopDrop;

import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.Intake.Velocity;
import frc.robot.subsystem.arm.Arm;
public class DropTeleopCube extends SuppliedCommand {
    public DropTeleopCube(Arm arm, Intake intake) {
        super(() -> switch(arm.getPosition()) {
            case AUTO_MID-> 
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID));
            case LOW->
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_LOW));
            case MID->
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.OUTTAKE));
                //     intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_MID));
            case HIGH->
                    intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CUBE_HIGH));
            case AUTO_HIGH,AUTO_HOLD,AUTO_INTAKE_LOW,AUTO_LOW,HOLD,
                    INTAKE_HIGH_DOUBLE_SUBSTATION,INTAKE_HIGH_SINGLE_SUBSTATION,
                    INTAKE_LOW,INTAKE_LOW_DROPPED,START -> intake.runOnce(()->intake.outtake());
        
            
        });
    }
}
