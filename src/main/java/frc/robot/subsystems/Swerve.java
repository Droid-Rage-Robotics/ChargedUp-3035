// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  
  private CANSparkMax left_front_drive = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax left_front_turn = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax right_front_drive = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax right_front_turn = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax left_rear_drive = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax left_rear_turn = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax right_rear_drive = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax right_rear_turn = new CANSparkMax(7, MotorType.kBrushless);
  
  public Swerve() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
