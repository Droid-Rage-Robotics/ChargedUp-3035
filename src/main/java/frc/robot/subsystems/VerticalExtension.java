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

public class VerticalExtension extends SubsystemBase {
  
  public enum VerticalLevel{
    GROUND,
    MID,
    HIGH,
    INTAKE
  }
  public static class VerticalSetPoints{//TODO: Positions
    private static int GROUND_POS = 0,
                MID_POS = 0,
                HIGH_POS = 0,
                INTAKE_POS  = 0;

  }
  private final CANSparkMax elevMotor, elevMotorTwo;
  private final DutyCycleEncoder elevEncoder;
  private final PIDController elevController;

    
  public VerticalExtension() {
    elevMotor = new CANSparkMax(0, MotorType.kBrushless);
  
    elevEncoder = new DutyCycleEncoder(0);  //TODO: Where is it plugged in?
    elevController = new PIDController(0, 0, 0);
    elevController.setTolerance(5);
    elevMotor.setIdleMode(IdleMode.kBrake);

    elevMotorTwo = new CANSparkMax(0, MotorType.kBrushless);//TODO: Where is it plugged in?
    elevMotorTwo.setIdleMode(IdleMode.kBrake);
    elevMotorTwo.follow(elevMotor, true);
    
  }


  public CommandBase toGround() {
    return runOnce(() -> elevController.setSetpoint(VerticalSetPoints.GROUND_POS));
  }
  public CommandBase toMid() {
    return runOnce(() -> elevController.setSetpoint(VerticalSetPoints.MID_POS));
  }
  public CommandBase toHigh() {
    return runOnce(() -> elevController.setSetpoint(VerticalSetPoints.HIGH_POS));
  }
  public CommandBase toIntakeHigh() {
    return runOnce(() -> elevController.setSetpoint(VerticalSetPoints.INTAKE_POS));
  }
  
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vertical Encoder Pos", elevEncoder.get());
  }


  @Override
  public void simulationPeriodic() {
  }
}
