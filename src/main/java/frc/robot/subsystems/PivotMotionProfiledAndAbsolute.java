package frc.robot.subsystems;

import com.revrobotics.SparkMaxAbsoluteEncoder;

public class PivotMotionProfiledAndAbsolute extends PivotMotionProfiled {
    SparkMaxAbsoluteEncoder absoluteEncoder;
    public PivotMotionProfiledAndAbsolute() {
        super();
        absoluteEncoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Math.PI * 2);
        absoluteEncoder.setInverted(false);
    }
    @Override
    protected double getPosition() {
        double position = absoluteEncoder.getPosition();
        encoderPositionWriter.write(position);
        return position;
    }
}