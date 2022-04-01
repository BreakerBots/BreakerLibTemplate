// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerveauto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDriveConfig;

/** Add your docs here. */
public class BreakerSwerveTrajectoryAuto extends CommandBase {
    SwerveControllerCommand controller;
    BreakerSwerveTrajectoryAutoConfig config;
    BreakerSwerveDrive drivetrain;
    Subsystem requiredSubsystem;
    private Trajectory[] trajectorysToFollow;
    private int currentTrajectory = 0;
    private int prevTrajectory = 0;
    private boolean commandIsFinished = false;
    private boolean stopAtEnd = false;
    BreakerSwerveTrajectoryAuto(BreakerSwerveTrajectoryAutoConfig config, Boolean stopAtEnd, Subsystem requiredSubsystem, Trajectory... trajectorysToFollow) {
        drivetrain = config.getDrivetrain();
        this.trajectorysToFollow = trajectorysToFollow;
        this.config = config;
        this.stopAtEnd = stopAtEnd;
        this.requiredSubsystem = requiredSubsystem;
    }

    @Override
    public void execute() {
        if (currentTrajectory != prevTrajectory) {
            try {
                controller = new SwerveControllerCommand(trajectorysToFollow[currentTrajectory], drivetrain::getOdometryPoseMeters, 
                    drivetrain.getConfig().getKinematics(), config.getxPosPID(), config.getyPosPID(), config.gettAngPID(), drivetrain::setRawModuleStates, requiredSubsystem);
                controller.schedule();
            } catch (Exception e) {
                commandIsFinished = true;
            }
        }
        prevTrajectory = currentTrajectory;
        if (controller.isFinished()) {
            currentTrajectory ++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (stopAtEnd) {
            drivetrain.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return commandIsFinished;
    }
}
