// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.util.BreakerMath;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerSwerveModule {
    private BreakerSwerveDriveConfig config;
    private PIDController drivePID;
    private PIDController anglePID;
    private SimpleMotorFeedforward driveFF;
    private WPI_TalonFX turnMotor;
    private WPI_TalonFX driveMotor;
    public BreakerSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX turnMotor, BreakerSwerveDriveConfig config) {
        this.config = config;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        anglePID = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
        drivePID = new PIDController(config.getModuleVelkP(), config.getModuleVelkI(), config.getModuleVelKd());
        driveFF = new SimpleMotorFeedforward(config.getVelFeedForwardKs(), config.getVelFeedForwardKv());
    }
    
    public void setModuleTarget(Rotation2d tgtAngle, double speedMetersPreSec) {
        turnMotor.set(anglePID.calculate(getModuleAngle(), tgtAngle.getDegrees()));
        driveMotor.set(drivePID.calculate(getModuleVelMetersPerSec(), speedMetersPreSec) + driveFF.calculate(speedMetersPreSec));
    }

    public void setModuleTarget(SwerveModuleState targetState) {
        turnMotor.set(anglePID.calculate(getModuleAngle(), targetState.angle.getDegrees()));
        driveMotor.set(drivePID.calculate(getModuleVelMetersPerSec(), targetState.speedMetersPerSecond));
    }

    public double getModuleAngle() {
        return (getTurnMotorTicks() / BreakerMath.getTicksPerRotation(2048, config.getTurnMotorGearRatioToOne()) * 360);
    }

    public double getTurnMotorTicks() {
        return turnMotor.getSelectedSensorPosition();
    }
    
    public double getModuleVelMetersPerSec() {
       return BreakerUnits.inchesToMeters(BreakerMath.ticksToInches(driveMotor.getSelectedSensorVelocity() * 10,
        BreakerMath.getTicksPerInch(2048, config.getDriveMotorGearRatioToOne(), config.getWheelDiameter())));
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelMetersPerSec(), Rotation2d.fromDegrees(getModuleAngle()));
    }

    public boolean atAngleSetpoint() {
        return anglePID.atSetpoint();
    }

    public boolean atVelSetpoint() {
        return drivePID.atSetpoint();
    }

    public boolean atSetModuleState() {
        return (atAngleSetpoint() && atVelSetpoint());
    }
}
