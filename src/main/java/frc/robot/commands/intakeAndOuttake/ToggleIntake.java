// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeAndOuttake;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.arm.Arm;
public class ToggleIntake extends SequentialCommandGroup {
    public ToggleIntake(Arm arm, Intake intake) {
        addCommands(
            // SuppliedCommand.create(() -> switch (arm.getPosition()) {
            //     case INTAKE_LOW, INTAKE_HIGH_DOUBLE_SUBSTATION, INTAKE_HIGH_SINGLE_SUBSTATION, AUTO_INTAKE_LOW -> intake.runOnce(()-> Commands.none());
            //     default -> intake.runOnce(()->intake.toggle(true));
            // }),
            intake.runOnce(()->intake.toggle(true)),
            arm.setPositionCommand(arm.getPosition())
        );
    }
}
