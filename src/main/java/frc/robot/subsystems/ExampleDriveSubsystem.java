// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerColorSensor;
import frc.robot.BreakerLib.devices.BreakerPigeon2;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDriveConfig;
import frc.robot.BreakerLib.util.BreakerMotorControl;


public class ExampleDriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleDriveSubsystem. */
  BreakerDiffDrive drivetrain;
  BreakerDiffDriveConfig driveConfig;

  WPI_TalonFX left1;
  WPI_TalonFX left2;
  WPI_TalonFX left3;
  WPI_TalonFX right1;
  WPI_TalonFX right2;
  WPI_TalonFX right3;

  WPI_TalonFX[] leftMotors;
  WPI_TalonFX[] rightMotors;

  public ExampleDriveSubsystem(BreakerPigeon2 pigeon2) {
    left1 = new WPI_TalonFX(0);
    left2 = new WPI_TalonFX(0);
    left3 = new WPI_TalonFX(0);
    right1 = new WPI_TalonFX(0);
    right2 = new WPI_TalonFX(0);
    right3 = new WPI_TalonFX(0);

    leftMotors = BreakerMotorControl.createMotorArray(left1, left2, left3);
    rightMotors = BreakerMotorControl.createMotorArray(right1, right2, right3);
    
    driveConfig = new BreakerDiffDriveConfig(2048, 0, 0, 0, 0, 0, 0, null, null);
    drivetrain = new BreakerDiffDrive(leftMotors, rightMotors, false, true, pigeon2, driveConfig);
  }

  public BreakerDiffDrive getBaseDrivetrain() {
    return drivetrain;
  }

  @Override
  public void periodic() {
    drivetrain.updateOdometry();
  }
}
