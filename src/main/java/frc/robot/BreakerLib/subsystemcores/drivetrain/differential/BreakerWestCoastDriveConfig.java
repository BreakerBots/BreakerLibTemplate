// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.differential;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.BreakerLib.util.BreakerMath;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerWestCoastDriveConfig {
    DifferentialDriveKinematics kinematics;
    private double robotTrackWidthInches;
    private double encoderTicks;
    private double ticksPerInch;
    private double gearRatioTo1;
    private double wheelDiameter;
    private double wheelCircumference;
    private double getTicksPerWheelRotation;
    private double feedForwardKs;
    private double feedForwardKv;
    private double feedForwardKa;
    private PIDController leftPID;
    private PIDController rightPID;
    public BreakerWestCoastDriveConfig(double encoderTicks, double gearRatioTo1, double wheelDiameter, 
        double feedForwardKs, double feedForwardKv, double feedForwardKa, double robotTrackWidthInches, PIDController leftPID, PIDController rightPID) {
        
        kinematics = new DifferentialDriveKinematics(BreakerUnits.inchesToMeters(robotTrackWidthInches));

        ticksPerInch = BreakerMath.getTicksPerInch(encoderTicks, gearRatioTo1, wheelDiameter);
        wheelDiameter = BreakerMath.getCircumferenceFromDiameter(wheelDiameter);
        getTicksPerWheelRotation = BreakerMath.getTicksPerRotation(encoderTicks, gearRatioTo1);

        encoderTicks = this.encoderTicks;
        gearRatioTo1 = this.gearRatioTo1;
        feedForwardKs = this.feedForwardKs;
        feedForwardKv = this.feedForwardKv;
        feedForwardKa = this.feedForwardKa;
        robotTrackWidthInches = this.robotTrackWidthInches;
        leftPID = this.leftPID;
        rightPID = this.rightPID;
    }

    public double getTicksPerInch() {
        return ticksPerInch;
    }

    public double getEncoderTicks() {
        return encoderTicks;
    }

    public double getGearRatioTo1() {
        return gearRatioTo1;
    }

    public double getGetTicksPerWheelRotation() {
        return getTicksPerWheelRotation;
    }

    public double getWheelCircumference() {
        return wheelCircumference;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getFeedForwardKa() {
        return feedForwardKa;
    }

    public double getFeedForwardKs() {
        return feedForwardKs;
    }

    public double getFeedForwardKv() {
        return feedForwardKv;
    }

    public double getRobotTrackWidthInches() {
        return robotTrackWidthInches;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public PIDController getLeftPID() {
        return leftPID;
    }

    public PIDController getRightPID() {
        return rightPID;
    }
}
