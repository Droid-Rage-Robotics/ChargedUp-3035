// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class MoveIntake1High extends SequentialCommandGroup {
    public MoveIntake1High(Elevator elevator, Pivot pivot) {
        addCommands(
            elevator.moveIntake1High(),
            pivot.moveIntake1High()
        );
    }
}
