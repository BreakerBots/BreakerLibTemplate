// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< Updated upstream:src/main/java/frc/robot/BreakerLib/subsystemcores/Drivetrain/BreakerWestCoastDrive.java
package frc.robot.BreakerLib.subsystemcores.Drivetrain;
=======
package frc.robot.BreakerLib.SubsystemCores.Drivetrain.WestCoastDrive;
>>>>>>> Stashed changes:src/main/java/frc/robot/BreakerLib/subsystemcores/Drivetrain/WestCoastDrive/BreakerWestCoastDrive.java

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.SubsystemCores.Drivetrain.BreakerGenericDrivetrain;

public class BreakerWestCoastDrive extends BreakerGenericDrivetrain {
  private WPI_TalonFX leftLead;
  private WPI_TalonFX rightLead;
  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;
  private DifferentialDrive diffDrive;
  private BreakerWestCoastDriveConfig driveConfig;

  public static WPI_TalonFX[] createMotorArray(WPI_TalonFX... controllers){
    return controllers;
  }
  
  /** Creates a new West Coast Drive. */
  public BreakerWestCoastDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR, BreakerWestCoastDriveConfig driveConfig) {
    leftDrive = new MotorControllerGroup(leftLead, leftMotors);
    leftDrive.setInverted(invertL);
    leftLead = leftMotors[0];

    rightDrive = new MotorControllerGroup(rightLead, rightMotors);
    rightDrive.setInverted(invertR);
    rightLead = rightMotors[0];

    diffDrive = new DifferentialDrive(leftDrive, rightDrive);

    driveConfig = this.driveConfig;
  }

  public void move(double netSpeed, double turnSpeed) {
    diffDrive.arcadeDrive(netSpeed, turnSpeed);
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

  public double getRightDriveInches() {
    return getRightDriveTicks() / driveConfig.getTicksPerInch();
  }

  public double getRightDriveTicks() {
    return rightLead.getSelectedSensorPosition();
  }
}
