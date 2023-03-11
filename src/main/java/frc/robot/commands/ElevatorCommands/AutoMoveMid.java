// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot2;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class AutoMoveMid extends SequentialCommandGroup {
    public AutoMoveMid(Elevator elevator, Pivot pivot) {
        addCommands(
            elevator.moveAutoMid(),
            pivot.moveMid()
        );
    }
}
