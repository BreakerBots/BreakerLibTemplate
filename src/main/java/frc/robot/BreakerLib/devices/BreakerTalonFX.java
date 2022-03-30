// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class BreakerTalonFX {
    private WPI_TalonFX falcon;

    public BreakerTalonFX(int deviceID) {
        falcon = new WPI_TalonFX(deviceID);
    }

    public BreakerTalonFX(int deviceID, String canbus) {
        falcon = new WPI_TalonFX(deviceID, canbus);
    }


}
