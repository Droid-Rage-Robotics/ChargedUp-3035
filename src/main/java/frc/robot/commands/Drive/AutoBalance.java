// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.utilities.ShuffleboardValueBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private Drive drive;
  // private WriteOnlyBoolean atSetpointWriter = new WriteOnlyBoolean(false, "PID Auto balance at positionn", Drive.class.getSimpleName());
  public AutoBalance(Drive drive) {
    super(
        new ProfiledPIDController(
            0.042, 
            0,
            0.0005,
            new TrapezoidProfile.Constraints(1, 1)),
        drive::getPitch,
        0,
        (output, setpoint) -> {
            // Use the output (and setpoint, if desired) here
            drive.drive(output, 0, 0);
          });

    addRequirements(drive);
    this.drive = drive;
    getController().setTolerance(1); //degrees
    // ComplexWidgetBuilder.create(getController(), "PID Auto balance controller", Drive.class.getSimpleName());

  }
  
  @Override
  public void initialize() {
    System.out.println("autobalance start");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // atSetpointWriter.set(getController().atSetpoint());
    return getController().atSetpoint();
  }

  // @Override
  // public void end(boolean interrupted) {
  //   System.out.println("autobalance end");
  //   drive.setModuleStates(new SwerveModuleState[] {
  //       new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4)),
  //       new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
  //       new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
  //       new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4))
  //   });
  // }
}