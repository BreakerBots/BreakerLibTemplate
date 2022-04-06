// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerPigeon2;
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

  Translation2d leftFrontModulePosition;
  Translation2d rightFrontModulePosition;
  Translation2d leftBackModulePosition;
  Translation2d rightBackModulePosition;

  BreakerSwerveDriveConfig driveConfig;

  BreakerSwerveDrive drivetrain;

  public ExampleSwerveDriveSubsystem(BreakerPigeon2 imu) {
    leftFrontModulePosition = new Translation2d(x, y);
    rightFrontModulePosition = new Translation2d(x, y);
    leftBackModulePosition = new Translation2d(x, y);
    rightBackModulePosition = new Translation2d(x, y);

    driveConfig = new BreakerSwerveDriveConfig(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, leftFrontModulePosition, rightFrontModulePosition, leftBackModulePosition, rightBackModulePosition);
    driveConfig.setPidTolerences(tolerences);

    driveLF = new WPI_TalonFX(0);
    turnLF = new WPI_TalonFX(0);
    driveRF = new WPI_TalonFX(0);
    turnRF = new WPI_TalonFX(0);
    driveLB = new WPI_TalonFX(0);
    turnLB = new WPI_TalonFX(0);
    driveRB = new WPI_TalonFX(0);
    turnRB = new WPI_TalonFX(0);

    leftFrontModule = new BreakerSwerveModule(driveLF, turnLF, driveConfig);
    rightFrontModule = new BreakerSwerveModule(driveRF, turnRF, driveConfig);
    leftBackModule = new BreakerSwerveModule(driveLB, turnLB, driveConfig);
    rightBackModule = new BreakerSwerveModule(driveRB, turnRB, driveConfig);

    drivetrain = new BreakerSwerveDrive(driveConfig, imu, leftFrontModule, rightFrontModule, leftBackModule, rightBackModule);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivetrain.updateOdometry();;
  }
}
