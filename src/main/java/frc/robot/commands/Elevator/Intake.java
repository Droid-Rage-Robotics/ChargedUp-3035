// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import frc.robot.subsystems.HorizontalExtension;
import frc.robot.subsystems.VerticalExtension;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GroundIntake extends CommandBase {
    private final HorizontalExtension hExtension;
    private final VerticalExtension vExtension;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public GroundIntake(HorizontalExtension hExtension, VerticalExtension vExtension) {
        this.hExtension = hExtension;
        this.vExtension = vExtension;
    	// Use addRequirements() here to declare subsystem dependencies.
    	addRequirements(hExtension, vExtension);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hExtension.toIntake();
        vExtension.toGround();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
