// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class BreakerMotorControle {
    
    public static void setTalonBreakMode(WPI_TalonFX motor, Boolean isEnabled) {
        motor.setNeutralMode((isEnabled ? NeutralMode.Brake : NeutralMode.Coast));
    }

    public static void setTalonBreakMode(WPI_TalonSRX motor, Boolean isEnabled) {
        motor.setNeutralMode((isEnabled ? NeutralMode.Brake : NeutralMode.Coast));
    }
}
