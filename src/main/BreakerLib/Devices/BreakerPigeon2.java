// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.Devices;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerMath;

public class BreakerPigeon2 extends SubsystemBase {
  private WPI_Pigeon2 pigeon;
  private double imuInvert;
  private double pitch;
  private double yaw;
  private double roll;
  private double [] wxyzDist = new double [4];
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;

  /** Creates a new PigeonIMU. */
  public BreakerPigeon2(int deviceID, boolean isInverted) {
    pigeon = new WPI_Pigeon2(deviceID);
    if (isInverted) {
      imuInvert = -1;
    } else {
      imuInvert = 1;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    pitch = BreakerMath.constrainAngle(pigeon.getPitch());
    yaw = BreakerMath.constrainAngle(pigeon.getYaw()) * imuInvert;
    roll = BreakerMath.constrainAngle(pigeon.getRoll());
    
    setName("IMU");
    addChild("Pigeon", pigeon);
    
  }

  /** Returns pitch angle within +- 360 degrees */
  public double getPitch() {
    return pitch;
  }

  /** Returns yaw angle within +- 360 degrees */
  public double getYaw() {
    return yaw;
  }

  /** Returns roll angle within +- 360 degrees */
  public double getRoll() {
    return roll;
  }

  /** Returns raw yaw, pitch, and roll angles in an array */
  public double[] getRawAngles() {
    double[] RawYPR = new double[3];
    pigeon.getYawPitchRoll(RawYPR);
    return RawYPR;
  }

  /** Resets yaw to 0 degrees */
  public void reset() {
    pigeon.setYaw(0);
  }

  public double getGyroRates(int arrayElement) {
    double[] rawRates = new double[3];
    pigeon.getRawGyro(rawRates);
    return rawRates[arrayElement];
  }

  public double getPitchRate() {
    return getGyroRates(0);
  }

  public double getYawRate() {
    return getGyroRates(1);
  }

  public double getRollRate() {
    return getGyroRates(2);
  }

  public short getRawAccelerometerVals(int arrayElement) {
    short[] accelVals = new short[2];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals[arrayElement];
  }

  public double getIns2AccelX() {
    return (frc.robot.BreakerLib.Util.BreakerMath.fixedToFloat(getRawAccelerometerVals(0), 14) * 7.721772);
  }

  public double getIns2AccelY() {
    return (frc.robot.BreakerLib.Util.BreakerMath.fixedToFloat(getRawAccelerometerVals(1), 14) * 7.721772);
  }

  public double getIns2AccelZ() {
    return (frc.robot.BreakerLib.Util.BreakerMath.fixedToFloat(getRawAccelerometerVals(2), 14) * 7.721772);
  }

  private void calculate4DDistance() {
   wxyzDist[0] = yaw;
   xSpeed += getIns2AccelX();
   ySpeed += getIns2AccelY();
   zSpeed += getIns2AccelZ();
   wxyzDist[1] += xSpeed;
   wxyzDist[2] += ySpeed;
   wxyzDist[3] += zSpeed;
  }

}
