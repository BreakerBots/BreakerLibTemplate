// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveModule;

public class ExampleSwerveDriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSwerveDriveSubsystem. */
  WPI_TalonFX driveLF;
  WPI_TalonFX turnLF;
  WPI_TalonFX driveRF;
  WPI_TalonFX turnRF;
  WPI_TalonFX driveLB;
  WPI_TalonFX turnLB;
  WPI_TalonFX driveRB;
  WPI_TalonFX turnRB;
 
  BreakerSwerveModule leftFrontModule;
  BreakerSwerveModule rightFrontModule;
  BreakerSwerveModule leftBackModule;
  BreakerSwerveModule rightBackModule;

  BreakerSwerveDriveConfig driveConfig;

  BreakerSwerveDrive drivetrain;

  public ExampleSwerveDriveSubsystem() {
    driveConfig = new BreakerSwerveDriveConfig(maxForwardVel, maxSidewaysVel, maxAngVel, moduleAnglekP, 
    moduleAnglekI, moduleAngleKd, moduleVelkP, moduleVelkI, moduleVelKd, turnMotorGearRatioToOne, 
    driveMotorGearRatioToOne, wheelDiameter, wheelPositionsRelativeToCenter);

    driveLF = new WPI_TalonFX(deviceNumber);
    turnLF = new WPI_TalonFX(deviceNumber);
    driveRF = new WPI_TalonFX(deviceNumber);
    turnRF = new WPI_TalonFX(deviceNumber);
    driveLB = new WPI_TalonFX(deviceNumber);
    turnLB = new WPI_TalonFX(deviceNumber);
    driveRB = new WPI_TalonFX(deviceNumber);
    turnRB = new WPI_TalonFX(deviceNumber);

    leftFrontModule = new BreakerSwerveModule(driveLF, turnLF, driveConfig);
    rightFrontModule = new BreakerSwerveModule(driveRF, turnRF, driveConfig);
    leftBackModule = new BreakerSwerveModule(driveLB, turnLB, driveConfig);
    rightBackModule = new BreakerSwerveModule(driveRB, turnRB, driveConfig);

    drivetrain = new BreakerSwerveDrive(driveConfig, leftFrontModule, rightFrontModule, leftBackModule, rightBackModule);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
