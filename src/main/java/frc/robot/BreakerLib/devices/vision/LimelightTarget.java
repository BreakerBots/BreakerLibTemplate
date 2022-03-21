// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class LimelightTarget {
    private int pipelineNum;
    private double targetHeight;
    private BreakerLimelight limelight;
    /** Creates the profile for the target of a limelight
     * @param targetHeight the height of the intended target form the ground
     * @param limelight the limelight used to track the intended target
     * @param pipelineNum the pipeline number (0 - 9) that the limelight needs to use to track the intended target
     */
    public LimelightTarget(double targetHeight, BreakerLimelight limelight, double pipelineNum) {
      targetHeight = this.targetHeight;
      pipelineNum = this.pipelineNum;
      limelight = this.limelight;
    }

    public double getTargetPipeline() {
      return pipelineNum;
    }

    public double getTargetOffsetX() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("tx").getDouble(0);
    }

    public double getTargetOffsetY() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ts").getDouble(0);
    }

    public double get3DTargetData() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("camtran").getDouble(0);
    }

    public double getTargetDistance() {
      double camHeight = targetHeight - limelight.getMountingHeight();
      double corTarAng = limelight.getMountingAngle() + getTargetOffsetY();
      return (camHeight / Math.tan(corTarAng));
    }
  }
