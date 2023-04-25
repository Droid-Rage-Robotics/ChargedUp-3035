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
    public final Color red = Color.kRed, 
                      batteryBlue = Color.kMidnightBlue,
                      orange = Color.kOrange, 
                      purple = Color.kPurple, //CUbe
                      yellow = Color.kOrangeRed, //COne
                      blue = Color.kBlue, //Decoration
                      green = Color.kGreen;
    public static Timer timer = new Timer();
    class SwitchLED{
      private boolean on = true;
      private double lastChange;
    }
    class FlashingColor{
      private long waitTime = 200, 
                    startTime = System.currentTimeMillis();
      private int stage = 0;
    }
    private SwitchLED switchLED;
    private FlashingColor flashingColor;

  //   private TrobotAddressableLEDPattern m_onPattern;
	// private TrobotAddressableLEDPattern m_offPattern;
	// private double m_interval = 2;
  
    // protected final ShuffleboardValue<String> intakeStateWriter = ShuffleboardValue.create(intakeState.name(), "IntakeState", Intake.class.getSimpleName())
    //     .build();
    public Light() {
      //TODO:Move most of these things to a command instead of the subsystem (very important)
      //TODO: Add a timeout for when Intake is first pressed
        
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(LED_COUNT);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
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


  public void setAlternatingColor(Color colorOne, Color colorTwo) {
    // For every pixel
    for (int i = 0; i < buffer.getLength(); i++) {
      if(i%2==0) {
        buffer.setLED(i, colorOne);

      } else{
        buffer.setLED(i, colorTwo);
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

    public void setAllColor(Color color) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }
  
    public void setAllColor(int r, int g, int b) {
      for (int i = 0; i < buffer.getLength(); i++) {
        setColor(i, r, g, b);
      }
    }
    public void setColor(int i,int r, int g, int b) {
        buffer.setRGB(i, r, g, b);
    }
    public void setColor(int i, Color color) {
      buffer.setLED(i, color);
  }

    public void elementIn() {
      setAllColor(red);
      // timer.reset();
      // timer.start();
    }

    public void trackElementLight() {
      switch (TrackedElement.get()) {
        case CONE:
            setAllColor(orange);
            return;
        case CUBE:
            setAllColor(purple);
            return;
      }
    }

    public void switchLeds() {
      double timestamp = Timer.getFPGATimestamp();
      if (timestamp - switchLED.lastChange > 1){
        switchLED.on = !switchLED.on;
        switchLED.lastChange = timestamp;
      }
      if (switchLED.on){
        setAlternatingColor(yellow, blue);
      } else {
        setAlternatingColor(blue, yellow);
      }
    }

    public void chaseLED( int m_offset) {//Doesn't work correctly
      Color[] m_Colors = {blue, yellow};
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

    public void flashingColors(Color colorOne, Color colorTwo){
      if (System.currentTimeMillis() - flashingColor.startTime >= flashingColor.waitTime) {
        for (int i = 0; i < buffer.getLength(); i++) {
            if (i % 3 == flashingColor.stage) {
                setColor(i, colorOne);
                continue;
            }
            setColor(i, colorTwo);
        }
        flashingColor.stage = flashingColor.stage + 1 > 3 ? 0 : flashingColor.stage + 1;
        flashingColor.startTime = System.currentTimeMillis();
      }
    }
}

