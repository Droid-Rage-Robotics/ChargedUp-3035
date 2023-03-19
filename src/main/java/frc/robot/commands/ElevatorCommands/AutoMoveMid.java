// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public class AutoMoveMid extends SequentialCommandGroup {
    public AutoMoveMid(Elevator elevator, Pivot pivot) {
        addCommands(
            elevator.moveAutoMid(),
            new WaitCommand(1.4),
            pivot.moveMid()
        );
    }
}
