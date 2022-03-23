// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.util.BreakerMath;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerSwerveModule {
    private BreakerSwerveDriveConfig config;
    private PIDController drivePID;
    private PIDController anglePID;
    private WPI_TalonFX turnMotor;
    private WPI_TalonFX driveMotor;
    public BreakerSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX turnMotor, BreakerSwerveDriveConfig config) {
        this.config = config;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        anglePID = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
        drivePID = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd());
        
    }
    
    public void setModuleTarget(Rotation2d tgtAngle, double speedMetersPreSec) {
        anglePID.calculate(getModuleAngle(), tgtAngle.getDegrees());
        drivePID.calculate(, speedMetersPreSec);
    }

    public double getModuleAngle() {
        return (getTurnMotorTicks() / BreakerMath.getTicksPerRotation(2048, config.getTurnMotorGearRatioToOne()) * 360);
    }

    public double getTurnMotorTicks() {
        return turnMotor.getSelectedSensorPosition();
    }
    
    public double getModuleVelMetersPerSec() {
        BreakerMath.ticksToInches(driveMotor.getSelectedSensorVelocity() * 10, BreakerMath.getTicksPerInch(2048, config.getDriveMotorGearRatioToOne(), ))
    }
}
