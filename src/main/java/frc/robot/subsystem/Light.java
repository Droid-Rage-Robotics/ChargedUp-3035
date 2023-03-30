package frc.robot.subsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {//TODO:Fix
    private Intake intake;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int m_rainbowFirstPixelHue = 0;
    private int LED_COUNT = 59;
    private final Color red, blue, green, yellow, purple, setColor;
    public Light(Intake intake) {
        this.intake = intake;
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        red = Color.kRed;
        blue = Color.kBlue;
        green = Color.kGreen;
        yellow = new Color(255, 255, 0);
        purple = Color.kPurple;
        setColor = Color.fromHSV(0,0,0);
        // spark = new Spark(1);
    }

    @Override
    public void periodic() {
      // rainbow();
      setColorType();//TODO:Why did this stop working?
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

    private void yellow() {
      // For every pixel
      for (int i = 0; i < buffer.getLength(); i++) {
        // final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
        // Set the value
        buffer.setHSV(i, 255,255,0);
      }
  }

      
    public void setColorType() {
      // if(intake.isElementIn()){
      //   setColor(green);
      //   return;
      // }
        // switch (TrackedElement.get()) {
        //     case CONE:
                yellow();// setColor(yellow);
        //         return;
        //     case CUBE:
        //         setColor(purple);
        //         return;
        // }
    }

    public void setColor(Color color) {
        for (var i = 0; i < LED_COUNT; i++) {
          // buffer.setLED(i, color);
          // rainbow();
          buffer.setHSV(i, 255,255,0);
          // buffer.setRGB(i, (int)color.red* 255, (int)color.blue* 255, (int)color.green* 255);
        }
      }
  
      public void setColor(int r, int g, int b) {
        for (var i = 0; i < LED_COUNT; i++) {
          buffer.setRGB(i, r, g, b);
        }
      }
    
    public void setRumble(int port, double value) {
      value = MathUtil.clamp(value, 0, 1);
      short rumbleValue = (short) (value * 65535);
      
      DriverStationJNI.setJoystickOutputs(
          (byte) port, 0, rumbleValue, rumbleValue);//Don't know what output does
      }
}

