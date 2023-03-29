// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.hal.DriverStationJNI;

/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver Station.
 *
 * <p>This class handles Xbox input that comes from the Driver Station. Each time a value is
 * requested the most recent value is returned. There is a single class instance for each controller
 * and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 */
public class XBoxControllerOwn  {
  public XBoxControllerOwn() {
    
  }
 
  public void setRumble(int port, double value) {
    value = MathUtil.clamp(value, 0, 1);
    short rumbleValue = (short) (value * 65535);
    
    DriverStationJNI.setJoystickOutputs(
        (byte) port, 0, rumbleValue, rumbleValue);
  }
}
