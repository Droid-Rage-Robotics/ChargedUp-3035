package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;

public class PIvot extends SubsystemBase {
    public static class Constants {
        public static final double GEAR_RATIO = 1 / 1;
        public static final double GEAR_DIAMETER_METERS = 0.0481; // 1.893 inches
        public static final double ROT_TO_METER = (GEAR_RATIO * Math.PI * GEAR_DIAMETER_METERS);
    }

    private enum ArmPos {
        START(0),
        INTAKELOW(0),//TODO:Change

        LOWCONE(0),
        LOWCUBE(0),

        MIDCONE(0),
        MIDCUBE(0),

        HIGHCONE(0),
        HIGHCUBE(0),

        INTAKEHIGH(0),
        HOLD(0),
        ;

        private final double position;

        private ArmPos(double position) {
            this.position = position;
        }
    }
    private final CANSparkMax pivotMotor;
    private final PIDController controller;
    private volatile ArmPos armPos;
    private final AbsoluteEncoder pivotAbsoluteEncoder;
    // private final double manualSpeed = 0.4;
    
    public PIvot() {
        pivotMotor = new CANSparkMax(18, MotorType.kBrushless);//TODO: Where is it plugged in?

        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle); //TODO:Test // 8192
        pivotAbsoluteEncoder.setPositionConversionFactor(Constants.ROT_TO_METER);
        // armAbsoluteEncoder.setInverted(false);
        // armMotor.getForwardLimitSwitch(null);//What does this do
  

        controller = new PIDController(0, 0, 0);
        controller.setTolerance(0.10); // meters

        setArmPosition(ArmPos.START);
    }

    @Override
    public void periodic() {
        DroidRageConstants.putNumber("Arm Position", pivotAbsoluteEncoder.getPosition());
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public double getTargetHeight() {
        return DroidRageConstants.getNumber("Arm Encoder"+ armPos.name(), pivotAbsoluteEncoder.getPosition());
    }

    private CommandBase setArmPosition(ArmPos position) {
        return runOnce(() -> {
            this.armPos = position;
            controller.setSetpoint(getTargetHeight());
            DroidRageConstants.putString("ArmPosition", position.name());
        });
    }
    public CommandBase setArmPosition(double position) {
        return runOnce(() -> {
            controller.setSetpoint(position);
            DroidRageConstants.putNumber("ArmPosition", position);
        });
    }

    public CommandBase moveIntakeLow() {
        return setArmPosition(ArmPos.INTAKELOW);
    }
    public CommandBase moveIntakeHigh() {
        return setArmPosition(ArmPos.INTAKEHIGH);
    }
    
    public CommandBase moveLow() {
        switch (TrackedElement.get()) {
            case CONE:
                return setArmPosition(ArmPos.LOWCONE);
            case CUBE:
            case NONE:
            default:
                return setArmPosition(ArmPos.LOWCUBE);
        }
    }

    public CommandBase moveMid() {
        switch (TrackedElement.get()) {
            case CONE:
                return setArmPosition(ArmPos.MIDCONE);
            case CUBE:
            case NONE:
            default:
                return setArmPosition(ArmPos.MIDCUBE);
        }
    }

    public CommandBase moveHigh() {
        switch (TrackedElement.get()) {
            case CONE:
                return setArmPosition(ArmPos.HIGHCONE);
            case CUBE:
            case NONE:
            default:
                return setArmPosition(ArmPos.HIGHCUBE);
        }
    }

    public CommandBase moveHold() {
        return setArmPosition(ArmPos.HOLD);
    }

    public CommandBase moveToPosition() {
        return runOnce(() -> pivotMotor.set(controller.calculate(getTargetHeight())));
    }
    public double getArmPosition(){
        return 0;
        // return armMotor.
        //TODO:FIX!!
    }
}  
