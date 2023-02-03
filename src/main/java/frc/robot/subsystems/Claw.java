// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  public static class ClawSetPoints{
    private static int OPEN_POS  = 0,
                      CLOSE_POS = 0;
  }
  private final CANSparkMax elevMotor;
  private final PIDController elevController;
  public Claw() {
    elevMotor = new CANSparkMax(0, MotorType.kBrushless);
    elevMotor.setIdleMode(IdleMode.kBrake);
    elevController = new PIDController(0, 0, 0);
    elevController.setTolerance(5);
  }

  public CommandBase openClaw() {
    return runOnce(
        () -> {
          elevController.setSetpoint(ClawSetPoints.OPEN_POS);
        });
  }
  public CommandBase closeClaw() {
    return runOnce(
        () -> {
          elevController.setSetpoint(ClawSetPoints.CLOSE_POS);
        });
  }

  
  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
