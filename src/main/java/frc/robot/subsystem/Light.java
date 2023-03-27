package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.TrackedElement.Element;

public class Light extends SubsystemBase {//TODO:Fix
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int m_rainbowFirstPixelHue = 0;
    private final Color red, blue, green, yellow, purple;
    public Light() {
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(60);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();


        red = Color.fromHSV(0,0,0);
        blue = Color.fromHSV(0,0,0);
        green = Color.fromHSV(0,0,0);
        yellow = Color.fromHSV(0,0,0);
        purple = Color.fromHSV(0,0,0);
        // spark = new Spark(1);
    }

    @Override
    public void periodic() {
        rainbow();
        led.setData(buffer);

    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }
  
    private void rainbow() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
          // Set the value
          buffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }


    private void setColor(Color color) {
        
    }
      
    public void setColorCargoType(Element element) {
        switch (element) {
            case CONE:
                setColor(yellow);
                break;
            case CUBE:
                setColor(purple);
                break;
            
        }
    }
}

