// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public class BreakerSwerveDriveConfig {
    // https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html

    private double maxForwardVel;
    private double maxSidewaysVel;
    private double maxAngleVel;
    private int moduleNum;
    private double moduleAnglekP;
    private double moduleAnglekI;
    private double moduleAngleKd;
    private double moduleVelkP;
    private double moduleVelkI;
    private double moduleVelKd;
    private double turnMotorGearRatioToOne;
    private double driveMotorGearRatioToOne;
    public double wheelDiameter;
    private double velFeedForwardKs;
    private double velFeedForwardKv;

    private SwerveDriveKinematics kinematics;
    /** The overall configuration for a Breaker Swerve Drive, must be passed in. */
    public BreakerSwerveDriveConfig(double maxForwardVel, double maxSidewaysVel, double maxAngVel, 
        double moduleAnglekP, double moduleAnglekI, double moduleAngleKd, double moduleVelkP,
        double moduleVelkI, double moduleVelKd, double turnMotorGearRatioToOne, double driveMotorGearRatioToOne,
        double wheelDiameter, double velFeedForwardKs, double velFeedForwardKv, Translation2d... wheelPositionsRelativeToCenter) {

        this.maxForwardVel = maxForwardVel;
        this.maxSidewaysVel = maxSidewaysVel;
        this.maxAngleVel = maxAngVel;
        this.moduleAngleKd = moduleAngleKd;
        this.moduleAnglekI = moduleAnglekI;
        this.moduleAnglekP = moduleAnglekP;
        this.moduleVelKd = moduleVelKd;
        this.moduleVelkI = moduleVelkI;
        this.moduleVelkP = moduleVelkP;
        this.wheelDiameter = wheelDiameter;
        this.turnMotorGearRatioToOne = turnMotorGearRatioToOne;
        this.driveMotorGearRatioToOne = driveMotorGearRatioToOne;
        this.velFeedForwardKs = velFeedForwardKs;
        this.velFeedForwardKv = velFeedForwardKv;

        moduleNum = wheelPositionsRelativeToCenter.length;
        kinematics = new SwerveDriveKinematics(wheelPositionsRelativeToCenter);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public double getMaxForwardVel() {
        return maxForwardVel;
    }

    public double getMaxSidewaysVel() {
        return maxSidewaysVel;
    }

    public double getMaxAngleVel() {
        return maxAngleVel;
    }

    public int getNumerOfModules() {
        return moduleNum;
    }
    
    public double getModuleVelkP() {
        return moduleVelkP;
    }

    public double getModuleVelkI() {
        return moduleVelkI;
    }

    public double getModuleVelKd() {
        return moduleVelKd;
    }

    public double getModuleAnglekP() {
        return moduleAnglekP;
    }

    public double getModuleAnglekI() {
        return moduleAnglekI;
    }

    public double getModuleAngleKd() {
        return moduleAngleKd;
    }

    public double getTurnMotorGearRatioToOne() {
        return turnMotorGearRatioToOne;
    }

    public double getDriveMotorGearRatioToOne() {
        return driveMotorGearRatioToOne;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getVelFeedForwardKs() {
        return velFeedForwardKs;
    }

    public double getVelFeedForwardKv() {
        return velFeedForwardKv;
    }
}
