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
    protected enum Position {
        OPEN(0),
        CLOSE(0),
        ;

        protected final double angleDegrees;

        private Position(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    private final CANSparkMax elevatorMotor;
    private final PIDController elevatorController;

    public Claw() {
        elevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorController = new PIDController(0, 0, 0);
        elevatorController.setTolerance(5);
    }

    @Override
    public void periodic() {}
  
    @Override
    public void simulationPeriodic() {}

    private CommandBase move(Position position) {
        return runOnce (() -> elevatorController.setSetpoint(position.angleDegrees));
    }
  
    public CommandBase openClaw() {
        return move(Position.OPEN);
    }
    public CommandBase closeClaw() {
        return move(Position.CLOSE);
    }
  
    
    
}
