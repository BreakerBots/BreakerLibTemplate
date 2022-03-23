// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.BreakerLib.devices.BreakerPigeon2;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerSwerveDriveOdometry {
    SwerveDriveOdometry odometer;
    BreakerPigeon2 pigeon2;
    SwerveDrivePoseEstimator poseEstimator;
    BreakerSwerveDrive drivetrain;

    public BreakerSwerveDriveOdometry(BreakerSwerveDriveConfig config, BreakerPigeon2 pigeon2, BreakerSwerveDrive drivetrain) {
        this.pigeon2 = pigeon2;
        this.drivetrain = drivetrain;
        odometer = new SwerveDriveOdometry(config.getKinematics(), Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
    }

    public void update() {
        odometer.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), drivetrain.getSwerveModuleStates());
    }

    public double[] getPositionInches() {
        double [] pos = new double [3];
        pos[0] = pigeon2.getRawAngles()[0];
        pos[1] = BreakerUnits.metersToInches(odometer.getPoseMeters().getX());
        pos[2] = BreakerUnits.metersToInches(odometer.getPoseMeters().getY());
        return pos;
    }


}
