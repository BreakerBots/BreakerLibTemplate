// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerRoboRio {
    private double prevTime;
    public BreakerRoboRio() {
    }

    public double getInterCycleTime() {
        double curTime = RobotController.getFPGATime(); // In microseconds
        double diffTime = curTime - prevTime;
        prevTime = curTime;
        diffTime = BreakerUnits.microsecondsToSeconds(diffTime);
        return diffTime;
    }

    public long getRobotTimeMS() {
        return RobotController.getFPGATime();
    }

    public double getRobotTimeSeconds() {
        return RobotController.getFPGATime();
    }
}
