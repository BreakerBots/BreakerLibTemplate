package frc.robot.BreakerLib.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.devices.BreakerPigeon2;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerWestCoastDrive;
import frc.robot.BreakerLib.util.BreakerUnits;

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

    /** Updates the odometer position. Use via {@link UpdateDiffDriveOdometer}. */
    public void update() {
        driveOdometer.update(yawToRotation2d(), BreakerUnits.inchesToMeters(drivetrain.getLeftDriveInches()), BreakerUnits.inchesToMeters(drivetrain.getRightDriveInches()));
    }

    /** Returns current 2d position as an array of doubles.
     * 
     * @return Array of X-Y-Angle position (in, in, deg).
     */
    public double[] getPosition() {
        Pose2d basePose = driveOdometer.getPoseMeters();
        double xPos = Units.metersToInches(basePose.getX());
        double yPos = Units.metersToInches(basePose.getY());
        double degPos = basePose.getRotation().getDegrees();
        return new double[]{xPos, yPos, degPos};
    }

    public Pose2d getPoseMeters() {
        return driveOdometer.getPoseMeters();
    }
}