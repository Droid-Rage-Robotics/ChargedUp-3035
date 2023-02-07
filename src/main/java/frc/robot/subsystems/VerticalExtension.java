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
    protected enum Position {
        GROUND(0),
        MID(0),
        HIGH(0),
        INTAKE(0),
        ;

        protected final double height_meters;

        private Position(double hieght_meters) {
            this.height_meters = hieght_meters;
        }
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Vertical Encoder Pos", elevEncoder.get());
    }
  
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    private CommandBase move(Position level) {
        return runOnce(() -> elevController.setSetpoint(level.height_meters));
    }

    public CommandBase moveGround() {
        return move(Position.GROUND);
    }

    public CommandBase moveMid() {
        return move(Position.MID);
    }

    public CommandBase moveHigh() {
        return move(Position.HIGH);
    }

    public CommandBase moveIntake() {
        return move(Position.INTAKE);
    }
}  
