package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class IntakeWithSparkPID extends IntakeWithPID {
    private SparkMaxPIDController intakeController;
    public IntakeWithSparkPID() {
        super();
        intakeController = intakeMotor.getPIDController();
        intakeController.setP(0.000173611);
        intakeController.setI(0);
        intakeController.setD(0);
        intakeController.setIZone(0);
        intakeController.setFF(0.000173611);
    }

    @Override
    public void periodic() {
        super.periodic();
        intakeController.setReference(getTargetVelocity(), ControlType.kVelocity);
    }
}
