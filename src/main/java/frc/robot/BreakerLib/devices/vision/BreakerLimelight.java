// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenaricDevice;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;

public class BreakerLimelight extends BreakerGenaricDevice {
  private double mountingAngle;
  private double mountingHeight;
  private String limelightName;
  private LimelightTarget currentTarget;
  private DeviceHealth currentHealth;
  private String faults = "none";
  /** Creates an new vision prossesing limelight
   * @param limelightName the network name of the limelight you are initializing
   */
  public BreakerLimelight(String limelightName) {
    limelightName = this.limelightName;
  }

  /** sets the limelights mounting position relative to the ground
   * @param mountingAngle the angle of the center of the limelights vision retive to the ground
   * @param mountingHeight the height that the center of the limelight camera is mounted in relative to the ground
  */
  public void setMountingPosition(double mountingAngle, double mountingHeight) {
    mountingAngle = this.mountingAngle;
    mountingAngle = this.mountingHeight;
  }

  /** Returns the number of limelight's the active pipeline (0-9) */
  public double getPipeline() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("getPipe").getDouble(0);
  }


  public void setPipeline(double pipeline) {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("pipeline").setNumber(pipeline);
  }


  public String getName() {
    return limelightName;
  }

  public void setTarget(LimelightTarget target) {
    if (currentTarget != target) {
      currentTarget = target;
      if (getPipeline() != target.getTargetPipeline()) {
        setPipeline(target.getTargetPipeline());
      }
    }
  }

  /** Returns an array with all single vision values the limelight can return
   * @return tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert, getpipe, tx0, ty0, ta0, ts0, tx1, ty1, ta1, ts1, tx2, ty2, ta2, ts2
   */
  public double[] getAllVisionData() {
    double[] info = new double[26];
    info[0] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").getDouble(0);
    info[1] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx").getDouble(0);
    info[2] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty").getDouble(0);
    info[3] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ta").getDouble(0);
    info[4] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ts").getDouble(0);
    info[5] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tl").getDouble(0);
    info[6] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tshort").getDouble(0);
    info[7] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tlong").getDouble(0);
    info[8] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("thor").getDouble(0);
    info[9] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tvert").getDouble(0);
    info[10] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("getpipe").getDouble(0);
    info[11] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx0").getDouble(0);
    info[12] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty0").getDouble(0);
    info[13] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ta0").getDouble(0);
    info[14] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ts0").getDouble(0);
    info[15] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx1").getDouble(0);
    info[16] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty1").getDouble(0);
    info[17] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ta1").getDouble(0);
    info[18] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ts1").getDouble(0);
    info[19] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx2").getDouble(0);
    info[20] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty2").getDouble(0);
    info[21] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ta2").getDouble(0);
    info[22] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ts2").getDouble(0);
    info[23] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("cx0").getDouble(0);
    info[24] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("cy0").getDouble(0);
    info[25] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("cx1").getDouble(0);
    info[26] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("cy1").getDouble(0);
    return info;
  }

  public boolean hasTarget() {
    return (getAllVisionData()[0] > 0);
  }

  public LimelightTarget getCurrentTarget() {
    return currentTarget;
  }

  public double getMountingHeight() {
      return mountingHeight;
  }

  public double getMountingAngle() {
      return mountingAngle;
  }

  @Override
  public void runSelfTest() {
    if (getAllVisionData()[5] == 0) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults = "limelightDisconnected";
    } else {
      currentHealth = DeviceHealth.NOMINAL;
      faults = "none";
    }
  }

  @Override
  public DeviceHealth getHealth() {
    return currentHealth;
  }

  @Override
  public String getFaults() {
    return faults;
  }

  @Override
  public String getDeviceName() {
    return limelightName;
  }
}
