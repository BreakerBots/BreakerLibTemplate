// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class BreakerSwerveDriveConfig {
    // https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html

    public static final double AXLE_WIDTH_DISTANCE = 0;
    public static final double AXLE_LENGTH_DISTANCE = 0;

    private double maxForwardVel;
    private double maxSidewaysVel;
    private double maxAngleVel;

    private SwerveDriveKinematics kinematics;

    public BreakerSwerveDriveConfig(double maxFwdVel, double maxSidewaysVel, double maxAngVel, Translation2d... wheelPositionsRelativeToCenter) {
        this.maxForwardVel = maxForwardVel;
        this.maxSidewaysVel = maxSidewaysVel;
        this.maxAngleVel = maxAngVel;
        kinematics = new SwerveDriveKinematics(wheelPositionsRelativeToCenter);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}
