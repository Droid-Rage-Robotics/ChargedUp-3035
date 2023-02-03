package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    public static class Constants {
        public static final Port PORT = Port.kMXP; // Dont use the Onboard i2c ports becuase they are prone to lockups
                                                   // This port should be on top of the Navx
        // public static final int RED_THRESHOLD = 0;
        public static final int BLUE_THRESHOLD = 0;
        public static final int GREEN_THRESHOLD = 0;
        public static final int DISTANCE_THRESHOLD = 0;
    }
    
    private final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.PORT);
    
    public ColorSensor() {}
    
    public boolean isInRange() { // Yellow
        return colorSensor.getProximity() > Constants.DISTANCE_THRESHOLD;
    }
    
    public boolean isCube() { //Purple
        return colorSensor.getBlue() > Constants.BLUE_THRESHOLD;
    }
    
    public boolean isCone() { // Yellow
        return colorSensor.getGreen() > Constants.GREEN_THRESHOLD;
    }
}
