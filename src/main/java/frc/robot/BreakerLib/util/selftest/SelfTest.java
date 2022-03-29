// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.selftest;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenaricDevice;

public class SelfTest extends SubsystemBase {
  /** Creates a new SelfTest. */
  private int cycleCount;
  private static List<BreakerGenaricDevice> devices = new ArrayList<BreakerGenaricDevice>();
  public SelfTest() {
  }

  public static void addDevice(BreakerGenaricDevice device) {
    devices.add(device);
  }

  @Override
  public void periodic() {
    if (cycleCount ++ % 250 == 0) {
      StringBuilder work = new StringBuilder(" SELF CHECK: ");
      List<BreakerGenaricDevice> faultDevices = new ArrayList<BreakerGenaricDevice>();
      for (BreakerGenaricDevice device: devices) {
        device.runSelfTest();
        if (device.getHealth() != DeviceHealth.NOMINAL) {
          faultDevices.add(device);
        }
      }
      if (faultDevices.size() != 0) {
        work.append(" SELF CHECK FAILED - FAULTS FOUND: ");
        for (BreakerGenaricDevice faultDiv: faultDevices) {
          work.append(" " + faultDiv.getDeviceName() + "-" + faultDiv.getFaults() + " ");
        }
      } else {
        work.append(" SELF CHECK PASSED ");
      }
    }
  }
}
