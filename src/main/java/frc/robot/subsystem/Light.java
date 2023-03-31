package frc.robot.subsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Light extends SubsystemBase {//TODO:Fix
    private Intake intake;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int m_rainbowFirstPixelHue = 0;
    private int LED_COUNT = 47;
    public final Color green, yellow, purple;
    public static Timer timer = new Timer();
    private final CommandXboxController driver;
    public Light(Intake intake, CommandXboxController driver) {//TODO:Move most of these things to a command instead of the subsystem (very important)
      //TODO: Add a timeout for when Intake is first pressed
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
        this.driver = driver;
        // spark = new Spark(1);
    }

    @Override
    public void periodic() {
      // setColorType();
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
      if (timer.get()<0.6){
        setRumble(driver, 0.8);
        return;
      }
      else if(intake.isElementIn() && driver.getRightTriggerAxis()>0.8){
        setColor(green);
        setRumble(driver, 0.8);
        timer.reset();
        timer.start();
        return;
      }
      else{
        setRumble(driver, 0);
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

    public void elementIn() {
      setColor(green);
      timer.reset();
      timer.start();
    }

    public void trackElementLight() {
      switch (TrackedElement.get()) {
        case CONE:
            setColor(yellow);
            return;
        case CUBE:
            setColor(purple);
            return;
      }
    }
    
    public void setRumble(CommandXboxController driver, double value) {
      value = MathUtil.clamp(value, 0, 1);
      short rumbleValue = (short) (value * 65535);

      CommandXboxController a = driver;
      a.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
      
      // DriverStationJNI.setJoystickOutputs(
      //     (byte) port, 0, rumbleValue, rumbleValue);//Don't know what output does
    }
}

