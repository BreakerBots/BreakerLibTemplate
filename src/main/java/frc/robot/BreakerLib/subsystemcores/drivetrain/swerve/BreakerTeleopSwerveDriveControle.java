// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.devices.BreakerPigeon2;

public class BreakerTeleopSwerveDriveControle extends CommandBase {
  /** Creates a new BreakerTeleopSwerveDriveControle. */
  private BreakerPigeon2 pigeon2;
  public BreakerTeleopSwerveDriveControle(BreakerPigeon2 pigeon2, BreakerSwerveDrive drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pigeon2 = pigeon2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
