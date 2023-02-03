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
  public static class VerticalSetPoints{
    private static int GROUND_POS = 0,
                MID_POS = 0,
                HIGH_POS = 0,
                INTAKE_POS  = 0;

  }
  private final CANSparkMax elevMotor, elevMotorTwo;
  private final DutyCycleEncoder elevEncoder;
  private final PIDController elevController;
  // private final int VerticalExtensionLocation = 0;
  //Not Sure which one to use
  // private final DigitalInput elevLimitSwitch;
  // private final LimitSwitchNormal limitSwitchNormal;
    
  /** Creates a new ExampleSubsystem. */
  public VerticalExtension() {
    elevMotor = new CANSparkMax(0, MotorType.kBrushless);
    elevMotorTwo = new CANSparkMax(0, MotorType.kBrushless);
    elevEncoder = new DutyCycleEncoder(0);  //Where is it plugged in?
    elevController = new PIDController(0, 0, 0);
    elevController.setTolerance(5);

    // elevLimitSwitch = new DigitalInput(2);
    // limitSwitchNormal = LimitSwitchNormal.NormallyOpen;

    elevMotor.setIdleMode(IdleMode.kBrake);
    elevMotorTwo.setIdleMode(IdleMode.kBrake);
    elevMotorTwo.follow(elevMotor, true);
    // VerticalExtensionLocation = Level.GROUND;
  }


  public CommandBase toGroundPos() {
    return runOnce(
        () -> {
          elevController.setSetpoint(VerticalSetPoints.GROUND_POS);
        });
  }
  public CommandBase toMidPos() {
    return runOnce(
        () -> {
          elevController.setSetpoint(VerticalSetPoints.MID_POS);
        });
  }
  public CommandBase toHighPos() {
    return runOnce(
        () -> {
          elevController.setSetpoint(VerticalSetPoints.HIGH_POS);
        });
  }
  public CommandBase toIntakePos() {
    return runOnce(
        () -> {
          elevController.setSetpoint(VerticalSetPoints.INTAKE_POS);
        });
  }
//   public boolean isDown() {
//     return elevLimitSwitch.get();
// // limitSwitchNormal.
//   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vertical Encoder Pos", elevEncoder.get());

    // if(elevLimitSwitch.get()){
    //   if (elevMotor.get()<0){
    //     elevMotor.set(0);
    //   }
    // }
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
