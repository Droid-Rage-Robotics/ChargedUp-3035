// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class MoveIntakeHigh extends CommandBase {
    private final Elevator elevator;
    private final Pivot pivot;

    public MoveIntakeHigh(Elevator elevator, Pivot pivot) {
    	this.elevator = elevator;
        this.pivot = pivot;
        
    	addRequirements(elevator, pivot);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            elevator.moveIntakeHigh(),
            pivot.moveIntakeHigh()
        ).execute();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
