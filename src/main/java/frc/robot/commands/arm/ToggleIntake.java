// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
public class ToggleIntake extends SequentialCommandGroup {
    public ToggleIntake(Arm arm, Intake intake) {
        addCommands(
            intake.runOnce(intake::toggle),
            arm.setPositionCommand(arm.getPosition())
        );
    }
}