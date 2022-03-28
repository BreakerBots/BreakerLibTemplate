// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.differential;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.subsystemcores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerMotorControl;
import frc.robot.BreakerLib.util.BreakerUnits;

public class BreakerDiffDrive extends BreakerGenericDrivetrain {
  private WPI_TalonFX leftLead;
  private WPI_TalonFX rightLead;
  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;
  private WPI_TalonFX[] leftMotors;
  private WPI_TalonFX[] rightMotors;
  private DifferentialDrive diffDrive;
  private BreakerDiffDriveConfig driveConfig;
  
  /** Creates a new West Coast Drive. */
  public BreakerDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR, BreakerDiffDriveConfig driveConfig) {
    leftDrive = new MotorControllerGroup(leftLead, leftMotors);
    leftDrive.setInverted(invertL);
    leftLead = leftMotors[0];

    rightDrive = new MotorControllerGroup(rightLead, rightMotors);
    rightDrive.setInverted(invertR);
    rightLead = rightMotors[0];

    diffDrive = new DifferentialDrive(leftDrive, rightDrive);

    driveConfig = this.driveConfig;
    leftMotors = this.leftMotors;
    rightMotors = this.rightMotors;
  }

  public void arcadeDrive(double netSpeed, double turnSpeed) {
    diffDrive.arcadeDrive(netSpeed, turnSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankMoveVoltage(double leftVoltage, double rightVoltage) {
    leftDrive.setVoltage(leftVoltage);
    rightDrive.setVoltage(rightVoltage);
    diffDrive.feed();
  }

  public void resetDriveEncoders() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }

  public double getLeftDriveTicks() {
    return leftLead.getSelectedSensorPosition();
  }

  public double getLeftDriveInches() {
    return getLeftDriveTicks() / driveConfig.getTicksPerInch();
  }

  public double getLeftDriveMeters() {
    return Units.inchesToMeters(getLeftDriveInches());
  }

  public double getRightDriveTicks() {
    return rightLead.getSelectedSensorPosition();
  }

  public double getRightDriveInches() {
    return getRightDriveTicks() / driveConfig.getTicksPerInch();
  }

  public double getRightDriveMeters() {
    return Units.inchesToMeters(getRightDriveInches());
  }

  public void setDrivetrainBrakeMode(boolean isEnabled) {
    for (WPI_TalonFX motorL: leftMotors) {
      BreakerMotorControl.setTalonBrakeMode(motorL, isEnabled);
    }
    for (WPI_TalonFX motorR: rightMotors) {
      BreakerMotorControl.setTalonBrakeMode(motorR, isEnabled);
    }
  }
  
  /** Returns an instance of the drivetrain's left side lead motor */
  public WPI_TalonFX getLeftLeadMotor() {
    return leftLead;
  }

  /** Returns an instance of the drivetrain's right side lead motor */
  public WPI_TalonFX getRightLeadMotor() {
    return rightLead;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return new SimpleMotorFeedforward(driveConfig.getFeedForwardKs(), driveConfig.getFeedForwardKv(), driveConfig.getFeedForwardKa());
  }

  public DifferentialDriveKinematics getKinematics() {
    return driveConfig.getKinematics();
  }

  public PIDController getLeftPIDController() {
    return driveConfig.getLeftPID();
  }

  public PIDController getRightPIDController() {
    return driveConfig.getRightPID();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( BreakerUnits.inchesToMeters((leftLead.getSelectedSensorVelocity() / driveConfig.getTicksPerInch()) * 10),
     BreakerUnits.inchesToMeters((rightLead.getSelectedSensorVelocity() / driveConfig.getTicksPerInch()) * 10));
  }

  @Override
  public void setOdometry() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void getOdometer() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void updateOdometry() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void getOdometryPosition() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void getOdometryPoseMeters() {
    // TODO Auto-generated method stub
    
  }
}
