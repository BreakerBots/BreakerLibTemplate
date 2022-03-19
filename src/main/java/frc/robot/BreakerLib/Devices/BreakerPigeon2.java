// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.Devices;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.Util.BreakerMath;

/* Good version of the CTRE Pigeon 2 class BAYBEEE! */
public class BreakerPigeon2 extends SubsystemBase {

  private WPI_Pigeon2 pigeon;

  private double imuInvert;

  private double pitch;
  private double yaw;
  private double roll;
  /* Coordinate array. w = angle, x = x-axis, y = y-axis*/
  private double[] wxy = new double[3];

  private double xSpeed;
  private double ySpeed;
  private double zSpeed;

  private double prevTime;
  private int cycleCount;
  private double XAccelBias = 0;
  private double YAccelBias = 0;

  /** Creates a new PigeonIMU object. */
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

    calculateAccelerometerBias();
    calculateGlobalPosition();
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

  /** Returns accelerometer value based on given index */
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

  public void calculateAccelerometerBias() {
    cycleCount ++;
    if (cycleCount < 150 && ((getIns2AccelY() < 2) && (getIns2AccelX() < 2))) {
      YAccelBias = BreakerMath.getAvg(YAccelBias, getRawIns2AccelY(), cycleCount);
      XAccelBias = BreakerMath.getAvg(XAccelBias, getRawIns2AccelX(), cycleCount);
    }
  }

  public double getAccelXBias() {
    if (cycleCount > 150) {
      return XAccelBias;
    } else {
      return 0d;
    }
  }

  public double getAccelYBias() {
    if (cycleCount > 150) {
      return YAccelBias;
    } else {
      return 0d;
    }
  }

  public short getRawAccelerometerVals(int arrayElement) {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals[arrayElement];
  }

  public double getRawIns2AccelX() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals(0), 14) * 7.721772);
  }

  public double getIns2AccelX() {
    return getRawIns2AccelX() - getAccelXBias();
  }

  public double getRawIns2AccelY() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals(1), 14) * 7.721772);
  }

  public double getIns2AccelY() {
    return getRawIns2AccelY() - getAccelYBias();
  }

  public double getRawIns2AccelZ() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals(2), 14) * 7.721772);
  }

  private void calculateGlobalPosition() {
    double curTime = RobotController.getFPGATime();
    double diffTime = curTime - prevTime;
    diffTime *= 0.000001;
   // double diffTime = 0.02;
    double radYaw = (yaw * (Math.PI / 180.0));
    // xSpeed += getIns2AccelX();
    ySpeed += getIns2AccelY();
    // zSpeed += getIns2AccelZ();
    wxy[0] = yaw;
    wxy[1] += ((ySpeed * Math.cos(radYaw)) * diffTime);
    wxy[2] += ((ySpeed * Math.sin(radYaw)) * diffTime);
    prevTime = curTime;
  }

  public double[] getGlobalPosition() {
    return wxy;
  }

  public double getGlobalPositionComponents(int arrayElement) {
    return wxy[arrayElement];
  }

  public void resetGlobalPosition() {
    wxy[1] = 0;
    wxy[2] = 0;
    ySpeed = 0;
    xSpeed = 0;
    zSpeed = 0;
  }

  public int getPigeonUpTime() { 
    return pigeon.getUpTime();
  }

  

}
