package frc.robot.subsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DroidRageConstants;
import frc.robot.subsystem.drive.DriveConstants;

public class Light extends SubsystemBase {//TODO:Fix
    private Intake intake;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int m_rainbowFirstPixelHue = 0;
    private int LED_COUNT = 59;
    private final Color green, yellow, purple;
    private static Timer timer = new Timer();
    public Light(Intake intake) {
        this.intake = intake;
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        
        green = Color.kGreen;
        yellow = new Color(255, 255, 0);
        purple = Color.kPurple;
        timer.start();
        // spark = new Spark(1);
    }

    @Override
    public void periodic() {
      setColorType(); //TODO:Why did this stop working?
      led.setData(buffer);
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }
  
    private void rainbow() {
        // For every pixel
        for (int i = 0; i < buffer.getLength(); i++) {
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

    

      
    public void setColorType() {
      if (timer.get()<3){
        return;
      }
      else if(intake.isElementIn()){
        setColor(green);
        setRumble(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT, 1);
        timer.reset();
        timer.start();
        return;
      }
      else{
        setRumble(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT, 0);
        switch (TrackedElement.get()) {
          case CONE:
              setColor(yellow);
              return;
          case CUBE:
              setColor(purple);
              return;
        }
      }
      
    }

    public void setColor(Color color) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }
  
    public void setColor(int r, int g, int b) {
      for (int i = 0; i < buffer.getLength(); i++) {
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

