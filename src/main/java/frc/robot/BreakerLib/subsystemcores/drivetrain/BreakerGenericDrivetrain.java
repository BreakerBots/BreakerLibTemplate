// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain;

/** Contianer class for methods common to all drivetrain types */
public abstract class BreakerGenericDrivetrain {
    
    public abstract void setOdometry();

    public abstract void getOdometer();

    public abstract void updateOdometry();

    public abstract void getOdometryPosition();

    public abstract void getOdometryPoseMeters();
}
