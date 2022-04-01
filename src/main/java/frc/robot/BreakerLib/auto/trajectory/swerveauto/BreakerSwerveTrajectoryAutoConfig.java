// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerveauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDrive;

/** Add your docs here. */
public class BreakerSwerveTrajectoryAutoConfig {
    private PIDController xPosPID;
    private PIDController yPosPID;
    private ProfiledPIDController tAngPID;
    private BreakerSwerveDrive drivetrain;
    private Constraints constraints;
    public BreakerSwerveTrajectoryAutoConfig(BreakerSwerveDrive drivetrain, double xPositionKp, double xPositionKi, 
        double xPositionKd, double yPositionKp, double yPositionKi, double yPositionKd,  
        double thetaAngleKp, double thetaAngleKi, double thetaAngleKd, double maxAllowedThetaVel, double maxAllowedThetaAccel) {
        xPosPID = new PIDController(xPositionKp, xPositionKi, xPositionKd);
        yPosPID = new PIDController(yPositionKp, yPositionKi, yPositionKd);
        constraints = new Constraints(maxAllowedThetaVel, maxAllowedThetaAccel);
        tAngPID = new ProfiledPIDController(thetaAngleKp, thetaAngleKi, thetaAngleKd, constraints);
        this.drivetrain = drivetrain;
    }

    public ProfiledPIDController gettAngPID() {
        return tAngPID;
    }

    public PIDController getxPosPID() {
        return xPosPID;
    }

    public PIDController getyPosPID() {
        return yPosPID;
    }

    public BreakerSwerveDrive getDrivetrain() {
        return drivetrain;
    }
}
