// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class Logger {;
    private Logger() {
        DataLogManager.logNetworkTables(false);
        DataLogManager.start();
    }
    
    public static void logEvent(String event) {
      DataLogManager.log(" EVENT: " + event);
    }

    public static void logError(String error) {
        DataLogManager.log(" ERROR: " + error);
    }

    public static void log(String message) {
        DataLogManager.log(message);
    }
}
