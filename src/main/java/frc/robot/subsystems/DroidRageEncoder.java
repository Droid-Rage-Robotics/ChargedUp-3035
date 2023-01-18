package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

public class DroidRageEncoder {
    private final CANCoder canCoder;
    private double positionConversionFactor;
    private double velocityConversionFactor;

    /* Cancoder */
    /* these can be changed with canCoder.configSetCustomParam
    eSensorInitStrategy(700),
	eSensorDirection(701),
	eMagnetOffset(702),
	eSensorSync(703),
	eAbsSensorRange(704), 
    */

    public DroidRageEncoder(int deviceNumber, String canbus) {
        canCoder = new CANCoder(0, canbus);
    }

    public DroidRageEncoder(int deviceNumber) {
        canCoder = new CANCoder(0);
    }

    public void setPositionConversionFactor(double factor) {
        this.positionConversionFactor = factor;
    }

    public void setVelocityConversionFactor(double factor) {
        this.velocityConversionFactor = factor;
    }

    public double getPosition() {
        return canCoder.getPosition() * positionConversionFactor;
    }

    public double getVelocity() {
        return canCoder.getVelocity() * velocityConversionFactor;
    }

    public double getVoltage() {
        return canCoder.getBusVoltage();
    }

    public void setPosition(double position) {
        canCoder.setPosition(position);
    }

    public int getDeviceID() {
        return canCoder.getDeviceID();
    }
}
