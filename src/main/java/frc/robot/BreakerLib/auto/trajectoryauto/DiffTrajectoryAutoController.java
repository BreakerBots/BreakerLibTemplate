// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectoryauto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;

/** Add your docs here. */
public class DiffTrajectoryAutoController {
    TrajectoryConfig Config;
    DifferentialDriveVoltageConstraint voltageConstraints;
    BreakerDiffDrive drivetrain;
    public DiffTrajectoryAutoController(BreakerDiffDrive drivetrain) {
        voltageConstraints = new DifferentialDriveVoltageConstraint(drivetrain.getFeedforward(), drivetrain.getKinematics(), 0);
        Config = new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
            Config.setKinematics(drivetrain.getKinematics());
            Config.addConstraint(voltageConstraints);
    }    

    public TrajectoryConfig getConfig() {
        return Config;
    }

    public void runTrajecotryPath() {
        
    }


}
