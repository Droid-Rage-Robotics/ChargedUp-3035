package frc.robot.subsystem.arm.pivot;

import com.revrobotics.SparkMaxAbsoluteEncoder;

public class PivotAbsolute extends PivotMotionProfiled {
    public static class Consants {
        public static double RADIANS_PER_ROTATION = Math.PI * 2;
        public static double OFFSET = Math.PI / 2;
    }
    SparkMaxAbsoluteEncoder absoluteEncoder;
    public PivotAbsolute() {
        super();
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Math.PI * 2);
        absoluteEncoder.setVelocityConversionFactor(Math.PI * 2 / 60);
        absoluteEncoder.setInverted(true);
        setTargetPosition(Consants.OFFSET);
    }
    
    @Override
    public double getEncoderPosition() {
        double position = (absoluteEncoder.getPosition() + Consants.OFFSET) % Consants.RADIANS_PER_ROTATION;
        encoderPositionWriter.write(position);
        return position;
    }

    @Override
    public double getEncoderVelocity() {
        double velocity = absoluteEncoder.getVelocity();
        encoderVelocityWriter.write(velocity);
        return velocity;
    }

    @Override
    public void resetEncoder() {
        absoluteEncoder.setZeroOffset(0);
        motor.burnFlash();
    }
}