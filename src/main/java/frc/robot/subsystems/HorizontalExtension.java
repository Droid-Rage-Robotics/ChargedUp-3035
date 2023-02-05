// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HorizontalExtension extends SubsystemBase {
  public enum HorizontalLevel{
    INTAKE,
    BOTTOM,
    MID,
    TOP
  }
  public static class HorizontalSetPoints{//TODO: Positions
    private static int INTAKE_POS  = 0,
                      BOTTOM_POS = 0,
                      MID_POS = 0,
                      TOP_POS = 0;
  }
  private final CANSparkMax elevMotor;
  private final DutyCycleEncoder elevEncoder;
  private final PIDController elevController;
  
  /** Creates a new ExampleSubsystem. */
  public HorizontalExtension() {
    elevMotor = new CANSparkMax(0, MotorType.kBrushless); //TODO: change
    elevEncoder = new DutyCycleEncoder(0);
    elevMotor.setIdleMode(IdleMode.kBrake);
    elevController = new PIDController(0, 0, 0);
    elevController.setTolerance(5);
  }

 
  public CommandBase toIntake() {
    return runOnce(() -> elevController.setSetpoint(HorizontalSetPoints.INTAKE_POS));
  }

  public CommandBase toBottom() {
    return runOnce(() -> elevController.setSetpoint(HorizontalSetPoints.BOTTOM_POS));
  }

  public CommandBase toMid() {
    return runOnce(() -> {elevController.setSetpoint(HorizontalSetPoints.MID_POS);});
  }

  public CommandBase toTop() {
    return runOnce(() -> elevController.setSetpoint(HorizontalSetPoints.TOP_POS));
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Horizontal Encoder Pos", elevEncoder.get());
  }

  @Override
  public void simulationPeriodic() {
    periodic();
  }
}
