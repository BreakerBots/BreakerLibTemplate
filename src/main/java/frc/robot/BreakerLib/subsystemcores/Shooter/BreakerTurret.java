// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.Shooter;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/** Add your docs here. */
public class BreakerTurret extends SubsystemBase {
    PIDController yawPID;
    PIDController pitchPID;
    public BreakerTurret(double yawKp, double yawKi, double yawKd, double yawPosTolerence, double yawVelTolerence, BaseMotorController yawMotor,
        double pitchKp, double pitchKi, double pitchKd, double pitchPosTolerence, double pitichVelTolerernce, BaseMotorController pitchMotor) {
        yawPID = new PIDController(yawKp, yawKi, yawKd);
        yawPID.setTolerance(yawPosTolerence, yawVelTolerence);
        pitchPID = new PIDController(pitchKp, pitchKi, pitchKd);
        pitchPID.setTolerance(pitchPosTolerence, pitichVelTolerernce);
    }
}
