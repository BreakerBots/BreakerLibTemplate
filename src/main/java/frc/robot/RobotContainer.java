// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.util.BreakerLog;
import frc.robot.BreakerLib.util.selftest.SelfTest;


public class RobotContainer {
  private BreakerAutoManager autoManager;
  public RobotContainer() {
    SelfTest.addDevices(null, null);
    BreakerLog.startLog(false);
    autoManager = new BreakerAutoManager(null, null);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // add your diriver imput triggered commands here
  }

  public void setDriveBreakMode(Boolean isEneabled) {
    // populate with your base drivetrain's set brake mode call
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoManager.getSelectedBaseCommandGroup();
  }
}
