// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
    public enum Level{
      GROUND,
      MID,
      HIGH,
      INTAKE
    }
    private final CANSparkMax elevMotor;
    private final DutyCycleEncoder elevEncoder;
    private final PIDController elevController;
    //Not Sure which one to use
    private final DigitalInput elevLimitSwitch;
    private final LimitSwitchNormal limitSwitchNormal;

    /** Creates a new ExampleSubsystem. */
    public Elevator() {
        elevMotor = new CANSparkMax(3, MotorType.kBrushless);
        elevEncoder = new DutyCycleEncoder(0);  //Where is it plugged in?
        elevController = new PIDController(0, 0, 0);

        elevLimitSwitch = new DigitalInput(2);
        limitSwitchNormal = LimitSwitchNormal.NormallyOpen;

        elevMotor.setIdleMode(IdleMode.kBrake);

    }

    
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
        SmartDashboard.putNumber("Encoeder Pos", elevEncoder.get());

        if(elevLimitSwitch.get()){
            if (elevMotor.get()<0){
                elevMotor.set(0);
            }
        }
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
