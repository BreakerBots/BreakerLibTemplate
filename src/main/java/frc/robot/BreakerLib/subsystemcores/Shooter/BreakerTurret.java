// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.Shooter;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class BreakerTurret {
    PIDController yawPID;
    PIDController pitchPID;
    public BreakerTurret() {
        yawPID = new PIDController(kp, ki, kd);
        pitchPID = new PIDController(kp, ki, kd);
    }
}
