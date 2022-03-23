// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.BreakerLib.subsystemcores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerUnits;

public class BreakerSwerveDrive extends BreakerGenericDrivetrain {
  private BreakerSwerveDriveConfig config;
  // [0] = frontLeft, [1] = frontRight, [2] = backLeft, [3] = backRight
  private SwerveModuleState[] targetModuleStates;
  SwerveModuleState[] currentModuleStates;

  private BreakerSwerveModule frontLeftModule;
  private BreakerSwerveModule frontRightModule;
  private BreakerSwerveModule backLeftModule;
  private BreakerSwerveModule backRightModule;
  /** Creates a new BreakerSwerveDrive. */
  public BreakerSwerveDrive(BreakerSwerveDriveConfig config, BreakerSwerveModule frontLeftModule, BreakerSwerveModule frontRightModule, BreakerSwerveModule backLeftModule, BreakerSwerveModule backRightModule) {
    this.config = config;
    this.frontLeftModule = frontRightModule;
    this.frontRightModule = frontRightModule;
    this.backLeftModule = backLeftModule;
    this.backRightModule = backRightModule;
  }

  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec);
    targetModuleStates = config.getKinematics().toSwerveModuleStates(speeds);
    frontLeftModule.setModuleTarget(targetModuleStates[0].angle, targetModuleStates[0].speedMetersPerSecond);
    frontRightModule.setModuleTarget(targetModuleStates[1].angle, targetModuleStates[1].speedMetersPerSecond);
    backLeftModule.setModuleTarget(targetModuleStates[2].angle, targetModuleStates[2].speedMetersPerSecond);
    backRightModule.setModuleTarget(targetModuleStates[3].angle, targetModuleStates[3].speedMetersPerSecond);
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
