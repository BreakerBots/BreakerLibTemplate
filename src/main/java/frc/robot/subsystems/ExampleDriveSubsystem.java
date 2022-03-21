// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerColorSensor;
import frc.robot.BreakerLib.odometry.BreakerDiffDriveOdometry;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerWestCoastDrive;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerWestCoastDriveConfig;

public class ExampleDriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleDriveSubsystem. */
  BreakerWestCoastDrive drivetrain;
  BreakerWestCoastDriveConfig driveConfig;
  WPI_TalonFX left1;
  WPI_TalonFX left2;
  WPI_TalonFX left3;
  WPI_TalonFX right1;
  WPI_TalonFX right2;
  WPI_TalonFX right3;
  WPI_TalonFX[] leftMotors;
  WPI_TalonFX[] rightMotors;
  
  public ExampleDriveSubsystem() {
    left1 = new WPI_TalonFX(0);
    left2 = new WPI_TalonFX(0);
    left3 = new WPI_TalonFX(0);
    right1 = new WPI_TalonFX(0);
    right2 = new WPI_TalonFX(0);
    right3 = new WPI_TalonFX(0);

    leftMotors = BreakerWestCoastDrive.createMotorArray(left1, left2, left3);
    rightMotors = BreakerWestCoastDrive.createMotorArray(right1, right2, right3);
    
    driveConfig = new BreakerWestCoastDriveConfig(0, 0, 0);
    drivetrain = new BreakerWestCoastDrive(leftMotors, rightMotors, false, true, driveConfig);
  }

  @Override
  public void periodic() {
    
  }
}
