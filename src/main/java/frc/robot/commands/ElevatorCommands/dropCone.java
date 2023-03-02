// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
public class dropCone extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Elevator elevator;
    private final Pivot pivot;
    private final Intake intake;

    public dropCone(Elevator elevator, Pivot pivot, Intake intake) {
    	this.elevator = elevator;
        this.pivot = pivot;
        this.intake = intake;
        
    	addRequirements(elevator, pivot, intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            elevator.dropVerticalElevator(),
            intake.openClaw(),
            intake.outtakeFor(1),
            elevator.moveInHorizontalElevator(),
            new WaitCommand(1),
            pivot.moveHold()
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
