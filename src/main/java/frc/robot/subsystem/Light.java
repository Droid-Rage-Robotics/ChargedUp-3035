package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {//TODO:Fix
    // private Intake intake;
    // private final CommandXboxController driver;
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private int m_rainbowFirstPixelHue = 0;
    private int LED_COUNT = 47;
    public final Color red, yellow, purple, orange, blue;
    public static Timer timer = new Timer();

  //   private TrobotAddressableLEDPattern m_onPattern;
	// private TrobotAddressableLEDPattern m_offPattern;
	// private double m_interval = 2;
	private boolean on = true;
	private double lastChange;
  
    // protected final ShuffleboardValue<String> intakeStateWriter = ShuffleboardValue.create(intakeState.name(), "IntakeState", Intake.class.getSimpleName())
    //     .build();
    public Light() {//TODO:Move most of these things to a command instead of the subsystem (very important)
      //TODO: Add a timeout for when Intake is first pressed
        
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        
        red = Color.kRed;
        yellow = Color.kYellow;
        // yellow = new Color(255, 255, 0);
        purple = Color.kPurple;
        orange = Color.kOrangeRed;
        blue = Color.kBlue;
        // timer.start();
        // this.intake = intake;
        // this.driver = driver;
        
    }

    @Override
    public void periodic() {
      // setColorType();
      // rainbow();
      led.setData(buffer);
    }
  
    @Override
    public void simulationPeriodic() {
        periodic();
    }
  
    public void rainbow() {
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

    public void orangeAndBlue() {
      // For every pixel
      for (int i = 0; i < buffer.getLength(); i++) {
        if(i%2==0) {
          buffer.setLED(i, orange);

        } else{
          buffer.setLED(i, blue);
        }
      }
  }

  public void blueAndOrange() {
    // For every pixel
    for (int i = 0; i < buffer.getLength(); i++) {
      if(i%2==0) {
        buffer.setLED(i, blue);

      } else{
        buffer.setLED(i, orange);
      }
    }
}
    
    // public void setColorType() {
    //   if (timer.get()<0.6){
    //     setRumble(driver, 0.8);
    //     return;
    //   }
    //   else if(intake.isElementIn() && driver.getRightTriggerAxis()>0.8){
    //     setColor(green);
    //     setRumble(driver, 0.8);
    //     timer.reset();
    //     timer.start();
    //     return;
    //   }
    //   else{
    //     setRumble(driver, 0);
    //     switch (TrackedElement.get()) {
    //       case CONE:
    //           setColor(yellow);
    //           return;
    //       case CUBE:
    //           setColor(purple);
    //           return;
    //     }
    //   }
    // }

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
      setColor(red);
      // timer.reset();
      // timer.start();
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

    public void switchLeds() {
      double timestamp = Timer.getFPGATimestamp();
      if (timestamp- lastChange > 1){
        on = !on;
        lastChange = timestamp;
      }
      if (on){
        orangeAndBlue();
      } else {
        blueAndOrange();
      }
    }

    public void chaseLED( int m_offset) {//Doesn't work correctly
      Color[] m_Colors = {blue, orange};
      int numberOfColors = m_Colors.length;
      int effectiveIndex;
      int colorIndex;
      int bufferLength = buffer.getLength();
      for (int index = 0; index < bufferLength; index++){
        effectiveIndex = (index + m_offset) % bufferLength;
        colorIndex =( index /2 )% numberOfColors;
        buffer.setLED(effectiveIndex, m_Colors[colorIndex]);
      }
  
      m_offset =(m_offset+1) %bufferLength;
    }
}

