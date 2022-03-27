// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.diffauto;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDriveOdometry;

/** Add your docs here. */
public class BreakerRamsete {
    private RamseteCommand ramsete;
    private RamseteController ramseteController;
    private BreakerDiffDrive drivetrain;
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private TrajectoryConfig config;
    private DifferentialDriveVoltageConstraint voltageConstraints;
    private BreakerDiffDriveOdometry odometry;
    public BreakerRamsete(Trajectory trajectoryToFollow, BreakerDiffDrive drivetrain, BreakerDiffDriveOdometry odometry, 
    Subsystem subsystemRequirements, double ramseteB, double ramseteZeta, double maxVel, double maxAccel, double maxVoltage){
        drivetrain = this.drivetrain;
        odometry = this.odometry;
        voltageConstraints = new DifferentialDriveVoltageConstraint(drivetrain.getFeedforward(), drivetrain.getKinematics(), maxVoltage);
        config = new TrajectoryConfig(maxVel, maxAccel);
            config.setKinematics(drivetrain.getKinematics());
            config.addConstraint(voltageConstraints);
        ramseteController = new RamseteController(ramseteB, ramseteZeta);
        ramsete = new RamseteCommand(trajectoryToFollow, odometry :: getPoseMeters, ramseteController, drivetrain.getFeedforward(), 
        drivetrain.getKinematics(), drivetrain :: getWheelSpeeds, drivetrain.getLeftPIDController(), drivetrain.getRightPIDController(), drivetrain :: tankMoveVoltage, subsystemRequirements);
    }
}