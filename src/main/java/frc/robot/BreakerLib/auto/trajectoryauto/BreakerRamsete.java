// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectoryauto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BreakerLib.odometry.BreakerDiffDriveOdometry;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;

/** Add your docs here. */
public class BreakerRamsete {
    RamseteCommand ramsete;
    RamseteController ramseteController;
    BreakerDiffDrive drivetrain;
    DifferentialDriveWheelSpeeds wheelSpeeds;
    BreakerDiffDriveOdometry odometry;
    public BreakerRamsete(BreakerDiffDrive drivetrain, BreakerDiffDriveOdometry odometry, Trajectory trajectoryToFollow, Subsystem subsystemRequirements){
        drivetrain = this.drivetrain;
        odometry = this.odometry;
        ramseteController = new RamseteController(b, zeta)
        ramsete = new RamseteCommand(trajectoryToFollow, odometry :: getPoseMeters, ramseteController, drivetrain.getFeedforward(), 
        drivetrain.getKinematics(), drivetrain :: getWheelSpeeds, drivetrain.getLeftPIDController(), drivetrain.getRightPIDController(), drivetrain :: tankMoveVoltage, subsystemRequirements);
    }
}
