// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
public class DropCube extends SequentialCommandGroup {
    public DropCube(Arm arm, Intake intake) {
        addCommands(
            intake.runOuttakeFor(1),
            Commands.waitSeconds(1),
            arm.setPositionCommand(Position.HOLD)
        );
    }
}
