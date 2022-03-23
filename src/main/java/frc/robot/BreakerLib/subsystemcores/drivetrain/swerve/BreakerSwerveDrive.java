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
  BreakerSwerveDriveConfig config;
  // [0] = frontLeft, [1] = frontRight, [2] = backLeft, [3] = backRight
  SwerveModuleState[] targetModuleStates;
  SwerveModuleState[] currentModuleStates;

  PIDController frontLeftAng;
  PIDController frontLeftVel;
  PIDController frontRightAng;
  PIDController forntRightVel;
  PIDController backLeftAng;
  PIDController backLeftVel;
  PIDController backRightAng;
  PIDController backRightVel;

  /** Creates a new BreakerSwerveDrive. */
  public BreakerSwerveDrive(BreakerSwerveDriveConfig config) {
    this.config = config;
    frontLeftAng = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
    frontLeftVel = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd());
    frontRightAng = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
    forntRightVel = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd());
    backLeftAng = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
    backLeftVel = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd());
    backRightAng = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
    backRightVel = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd()); 
  }

  public void updateCurrentModuleStates() {
    currentModuleStates[0] = new SwerveModuleState(speedMetersPerSecond, angle);
    currentModuleStates[1] = new SwerveModuleState(speedMetersPerSecond, angle);
    currentModuleStates[2] = new SwerveModuleState(speedMetersPerSecond, angle);
    currentModuleStates[3] = new SwerveModuleState(speedMetersPerSecond, angle);
  }

  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec);
    targetModuleStates = config.getKinematics().toSwerveModuleStates(speeds);
    calculateFrontLeftPID(targetModuleStates[0].angle, targetModuleStates[0].speedMetersPerSecond);
    calculateFrontRightPID(targetModuleStates[1].angle, targetModuleStates[1].speedMetersPerSecond);
    calculateBackLeftPID(targetModuleStates[2].angle, targetModuleStates[2].speedMetersPerSecond);
    calculateBackRightPID(targetModuleStates[3].angle, targetModuleStates[3].speedMetersPerSecond);
  }

  private void calculateFrontLeftPID(Rotation2d tgtAngle, double speedMetersPreSec) {
    frontLeftAng.calculate(measurement, tgtAngle);
    frontLeftVel.calculate(measurement, BreakerUnits.metersToInches(speedMetersPreSec));
  }

  private void calculateFrontRightPID(Rotation2d tgtAngle, double speedMetersPreSec) {
    frontLeftAng.calculate(measurement, tgtAngle);
    frontLeftVel.calculate(measurement, BreakerUnits.metersToInches(speedMetersPreSec));
  }

  private void calculateBackLeftPID(Rotation2d tgtAngle, double speedMetersPreSec) {
    frontLeftAng.calculate(measurement, tgtAngle);
    frontLeftVel.calculate(measurement, BreakerUnits.metersToInches(speedMetersPreSec));
  }

  private void calculateBackRightPID(Rotation2d tgtAngle, double speedMetersPreSec) {
    frontLeftAng.calculate(measurement, tgtAngle);
    frontLeftVel.calculate(measurement, BreakerUnits.metersToInches(speedMetersPreSec));
  }



}
