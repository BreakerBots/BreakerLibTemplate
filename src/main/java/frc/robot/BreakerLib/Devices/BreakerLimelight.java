// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.Devices;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakerLimelight extends SubsystemBase {
  private double mountingAngle;
  private double mountingHeight;
  private double targetHeight;
  private String limelightName;
  private LimelightTarget currentTarget;
  /** Creates a new BreakerLimelight. */
  public BreakerLimelight(String limelightName) {
    limelightName = this.limelightName;
  }

  public void setMountingPosition(double mountingAngle, double mountingHeight) {
    mountingAngle = this.mountingAngle;
    mountingAngle = this.mountingHeight;
  }

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

  /** WIP */
  public double[] getAllVisionData() {
    double[] info = new double[28];
    info[0] = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("").getDouble(0);
    return info;
  }

  public LimelightTarget getCurrentTarget() {
    return currentTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public class LimelightTarget {
    private int pipelineNum;
    private double targetHeight;
    private BreakerLimelight limelight;
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

    public double getTargetDistance() {
      double camHeight = Constants.HUB_HEIGHT_INS - Constants.SHOOT_CAM_HEIGHT;
      double corTarAng = Constants.SHOOT_CAM_ANG + getTargetOffsetY();
      return (camHeight / Math.tan(corTarAng));
    }
  }
}
