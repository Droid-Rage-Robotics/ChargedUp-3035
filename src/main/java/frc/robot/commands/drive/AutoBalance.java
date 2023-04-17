// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystem.Light;
import frc.robot.subsystem.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {//TODO: Add a TImeout to lockwheels
  /** Creates a new AutoBalance. */
  // private static DriverStation driverStation;
  private Drive drive;
  private Light light;
  // private Timer timer;
  // private WriteOnlyBoolean atSetpointWriter = new WriteOnlyBoolean(false, "PID Auto balance at positionn", Drive.class.getSimpleName());
  public AutoBalance(Drive drive, Light light) {//TODO:Test
    
    super(
        new ProfiledPIDController(
            0.03, 
            0,
            0.0000,
            new TrapezoidProfile.Constraints(1, 1)),
        drive::getPitch,
        0,
        (output, setpoint) -> {
            // Use the output (and setpoint, if desired) here
            drive.drive(output, 0, 0);
          });

    addRequirements(drive);
    this.drive = drive;
    this.light = light;
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
      light.setAllColor(light.red);
    // final int time = driverStation.getMatchTime();
    // if(isMatchTime()){
    //   return true;
    // }

    // atSetpointWriter.set(getController().atSetpoint());
    if(getController().atSetpoint()){
      light.setAllColor(light.green);
    }
    return getController().atSetpoint();
  }

  // public boolean isMatchTime(){//TODO:test
  //   return DriverStation.getMatchTime()<2;
  // }

}