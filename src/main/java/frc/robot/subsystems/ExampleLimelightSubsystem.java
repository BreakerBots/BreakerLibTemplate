// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.vision.BreakerLimelight;
import frc.robot.BreakerLib.devices.vision.LimelightTarget;
import frc.robot.BreakerLib.util.selftest.SelfTest;

public class ExampleLimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleLimelightSubsystem. */
  BreakerLimelight limelight;
  LimelightTarget targetA;
  LimelightTarget targetB;
  public ExampleLimelightSubsystem() {
    limelight = new BreakerLimelight("limelight");
    SelfTest.addDevice(limelight);
    targetA = new LimelightTarget(102, limelight, 0);
    targetB = new LimelightTarget(15, limelight, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
