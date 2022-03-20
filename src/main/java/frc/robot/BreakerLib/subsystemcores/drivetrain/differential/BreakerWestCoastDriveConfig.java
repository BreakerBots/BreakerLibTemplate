// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.differential;

import frc.robot.BreakerLib.util.BreakerMath;

/** Add your docs here. */
public class BreakerWestCoastDriveConfig {
    private double encoderTicks;
    private double ticksPerInch;
    private double gearRatioTo1;
    private double wheelDiameter;
    private double wheelCircumference;
    private double getTicksPerWheelRotation;
    BreakerWestCoastDriveConfig(double encoderTicks, double gearRatioTo1, double wheelDiameter) {
        ticksPerInch = BreakerMath.getTicksPerInch(encoderTicks, gearRatioTo1, wheelDiameter);
        wheelDiameter = BreakerMath.getCircumferenceFromDiameter(wheelDiameter);
        getTicksPerWheelRotation = BreakerMath.getTicksPerRotation(encoderTicks, gearRatioTo1);
        encoderTicks = this.encoderTicks;
        gearRatioTo1 = this.gearRatioTo1;
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
}
