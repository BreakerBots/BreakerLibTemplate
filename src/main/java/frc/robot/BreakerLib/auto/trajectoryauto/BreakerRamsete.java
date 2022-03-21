// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectoryauto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.BreakerLib.odometry.BreakerDiffDriveOdometry;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerWestCoastDrive;

/** Add your docs here. */
public class BreakerRamsete {
    RamseteCommand ramsete;
    RamseteController ramseteController;
    BreakerWestCoastDrive drivetrain;
    DifferentialDriveWheelSpeeds wheelSpeeds;
    BreakerDiffDriveOdometry odometry;
    public BreakerRamsete(BreakerWestCoastDrive drivetrain, BreakerDiffDriveOdometry odometry, Trajectory trajectoryToFollow){
        drivetrain = this.drivetrain;
        odometry = this.odometry;
        wheelSpeeds = new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond)
        ramseteController = new RamseteController(b, zeta)
        ramsete = new RamseteCommand(trajectoryToFollow, odometry :: getPoseMeters, ramseteController, drivetrain.getFeedforward(), 
        drivetrain.getKinematics(), wheelSpeeds, drivetrain.getLeftPIDController(), drivetrain.getRightPIDController(), drivetrain :: tankMoveVoltage, drivetrain);
    }
}
