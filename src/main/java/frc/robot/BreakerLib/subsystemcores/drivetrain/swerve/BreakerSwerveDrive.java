// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.subsystemcores.drivetrain.BreakerGenericDrivetrain;

public class BreakerSwerveDrive extends BreakerGenericDrivetrain {
  private BreakerSwerveDriveConfig config;
  // [0] = frontLeft, [1] = frontRight, [2] = backLeft, [3] = backRight
  private SwerveModuleState[] targetModuleStates;
  SwerveModuleState[] currentModuleStates;

  private BreakerSwerveModule frontLeftModule;
  private BreakerSwerveModule frontRightModule;
  private BreakerSwerveModule backLeftModule;
  private BreakerSwerveModule backRightModule;
  /** Constructs a new swerve based drivetrain
   * @param config - the confiuration values for the drivetrain's charicotristics and behavor, passed in as a "BreakerSwerveDriveConfig" object
   * @param swerveModules - The four swerve drive modules that make up the drivetrain, must be passed in the same order shown below
   */
  public BreakerSwerveDrive(BreakerSwerveDriveConfig config, BreakerSwerveModule frontLeftModule, BreakerSwerveModule frontRightModule, BreakerSwerveModule backLeftModule, BreakerSwerveModule backRightModule) {
    this.config = config;
    this.frontLeftModule = frontRightModule;
    this.frontRightModule = frontRightModule;
    this.backLeftModule = backLeftModule;
    this.backRightModule = backRightModule;
  }

  /** Standard drivetrain movement command, specifyes robot speed in each axis including robot rotation (radian per second). */
  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec);
    targetModuleStates = config.getKinematics().toSwerveModuleStates(speeds);
    frontLeftModule.setModuleTarget(targetModuleStates[0]);
    frontRightModule.setModuleTarget(targetModuleStates[1]);
    backLeftModule.setModuleTarget(targetModuleStates[2]);
    backRightModule.setModuleTarget(targetModuleStates[3]);
  }

  public void moveWithPrecentImput(double forwardPercent, double horizontalPercent, double turnPercent) {
    move((forwardPercent * config.getMaxForwardVel()), (horizontalPercent * config.getMaxSidewaysVel()), (turnPercent * config.getMaxAngleVel()));
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    currentModuleStates[0] = frontLeftModule.getModuleState();
    currentModuleStates[1] = frontRightModule.getModuleState();
    currentModuleStates[2] = backLeftModule.getModuleState();
    currentModuleStates[3] = backRightModule.getModuleState();
    return currentModuleStates;
  }


}
