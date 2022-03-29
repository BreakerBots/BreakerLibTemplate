// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;

/** Contianer class for methods common to all drivetrain types */
public abstract class BreakerGenericDrivetrain {
    
    public abstract void setOdometry(Pose2d poseMeters, double gyroAngle);

    public abstract Object getOdometer();

    public abstract void updateOdometry();

    public abstract double[] getOdometryPosition();

    public abstract Pose2d getOdometryPoseMeters();
}
