package frc.robot.BreakerLib.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.Devices.BreakerPigeon2;
import frc.robot.BreakerLib.SubsystemCores.BreakerWestCoastDrive;

public class BreakerDiffDriveOdometry{

    private DifferentialDriveOdometry driveOdometer;
    private BreakerWestCoastDrive drivetrain;
    private BreakerPigeon2 pigeon2;

    public BreakerDiffDriveOdometry(BreakerWestCoastDrive drivetrain, BreakerPigeon2 pigeon2) {
        driveOdometer = new DifferentialDriveOdometry(new Rotation2d(pigeon2.getYaw()));
        this.drivetrain = drivetrain;
        this.pigeon2 = pigeon2;
    }

    private Rotation2d yawToRotation2d() {
        return new Rotation2d(pigeon2.getYaw());
    }

    public void update() {
        driveOdometer.update(yawToRotation2d(), drivetrain.getLeftDriveTicks(), drivetrain.getRightDriveTicks());
    }

    public double[] getPosition() {
        Pose2d basePose = driveOdometer.getPoseMeters();
        xPos = Units.metersToInches(basePose.getX());
    }
}