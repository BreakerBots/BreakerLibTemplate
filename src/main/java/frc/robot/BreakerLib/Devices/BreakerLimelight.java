// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.Devices;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakerLimelight extends SubsystemBase {
  private double mountingAngle;
  private double mountingHeight;
  private double targetHeight;
  private String limelightName;
  /** Creates a new BreakerLimelight. */
  public BreakerLimelight(String limelightName) {
    limelightName = this.limelightName;
  }

  public void setMountingPosition(double mountingAngle, double mountingHeight) {
    mountingAngle = this.mountingAngle;
    mountingAngle = this.mountingHeight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public class LimelightTarget {
    public LimelightTarget(double targetHeight, double target) {
      
    }
  }
}
