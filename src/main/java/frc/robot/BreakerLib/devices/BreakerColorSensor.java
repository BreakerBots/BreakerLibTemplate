// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;

public class BreakerColorSensor extends BreakerGenaricDevice {
  /** Creates a new BreakerColorSensor. */
  private ColorSensorV3 colorSensor;
  public BreakerColorSensor(Port i2cPort) {
    colorSensor = new ColorSensorV3(i2cPort);
  }

  public Color getColor() {
    return colorSensor.getColor();
  }

  public boolean compareColors(Color comparisionColor) {
    return (comparisionColor == colorSensor.getColor());
  }

  public int[] getRawColorsADC() {
    int[] colorVals = new int[4];
    colorVals[0] = colorSensor.getRawColor().red;
    colorVals[1] = colorSensor.getRawColor().green;
    colorVals[2] = colorSensor.getRawColor().blue;
    colorVals[3] = colorSensor.getRawColor().ir;
    return colorVals;
  }

  @Override
  public void runSelfTest() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public DeviceHealth getHealth() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public String getFaults() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public String getDeviceName() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean hasFault() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setDeviceName(String newName) {
    // TODO Auto-generated method stub
    
  }

  
}
