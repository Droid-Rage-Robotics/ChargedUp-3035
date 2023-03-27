// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystem.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {//TODO: Add a TImeout to lockwheels
  /** Creates a new AutoBalance. */
  // private static DriverStation driverStation;
  private Drive drive;
  // private Timer timer;
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
    getController().setTolerance(0.5); //degrees
    // ComplexWidgetBuilder.create(getController(), "PID Auto balance controller", Drive.class.getSimpleName());

  }
  
  @Override
  public void initialize() {
    System.out.println("autobalance start");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // final int time = driverStation.getMatchTime();
    if(isMatchTime()){
      // timer.stop();
      return true;
    }

    // atSetpointWriter.set(getController().atSetpoint());
    return getController().atSetpoint();
  }

  public boolean isMatchTime(){//TODO:test
    return DriverStation.getMatchTime()<2;
  }

}