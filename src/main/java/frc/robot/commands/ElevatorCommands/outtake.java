// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.TrackedElement;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class outtake extends CommandBase {
    private final Elevator elevator;
    private final Pivot pivot;
    private final Intake intake;

    public outtake(Elevator elevator, Pivot pivot, Intake intake) {
    	this.elevator = elevator;
        this.pivot = pivot;
        this.intake = intake;
        
    	addRequirements(elevator, pivot, intake);

        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(TrackedElement.Element.CONE==TrackedElement.get()){
            new dropCone(elevator, pivot, intake);
        } else if(TrackedElement.Element.CUBE ==TrackedElement.get()){
            new dropCube(elevator, pivot, intake);
        }
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
