// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.Shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.cosmetic.FalconOrchestra;
import frc.robot.BreakerLib.util.BreakerMath;
import frc.robot.BreakerLib.util.BreakerUnits;

/** Add your docs here. */
public class BreakerFlywheel extends SubsystemBase {
    private PIDController flyPID;
    private SimpleMotorFeedforward flyFF;
    private boolean runFlywheel = false;
    private double flywheelTargetRSU = 0;
    private MotorControllerGroup flywheel;
    private WPI_TalonFX rFlyMotor;
    private WPI_TalonFX lFlyMotor;
    public BreakerFlywheel(WPI_TalonFX leftFlywheelMotor, WPI_TalonFX rightFlywheelMotor, double flywheelKp, double flywheelKi, double flywheelKd, double flywheelPosTol, double flywheelVelTol,
        double flywheelKs, double flywheelKv, double flywheelKa) {
        flyPID = new PIDController(flywheelKp, flywheelKi, flywheelKd);
        flyPID.setTolerance(flywheelPosTol, flywheelVelTol);
        flyFF = new SimpleMotorFeedforward(flywheelKs, flywheelKv, flywheelKa);
        flywheel = new MotorControllerGroup(leftFlywheelMotor, rightFlywheelMotor);
        lFlyMotor = leftFlywheelMotor;
        rFlyMotor = rightFlywheelMotor;
    }

    public void setFlywheelSpeed(double flywheelTargetSpeedRPM) {
        runFlywheel = true;
        flywheelTargetRSU = BreakerUnits.RPMtoFalconRSU(flywheelTargetSpeedRPM);
    }

    public double getFlywheelVelRSU() {
        return lFlyMotor.getSelectedSensorVelocity();
    }

    public double getFlywheelTargetVelRSU() {
        return flywheelTargetRSU;
    }
    public void stopFlywheel() {
        runFlywheel = false;
        flywheel.set(0);
    }

    public void startFlywheel() {
        runFlywheel = true;
    }

    private void runFlywheel() {
        if (runFlywheel) {
        double flySetSpd = flyPID.calculate(getFlywheelVelRSU(), flywheelTargetRSU) + (flyFF.calculate(flywheelTargetRSU) / lFlyMotor.getBusVoltage());
        flywheel.set(flySetSpd);
        }
    }

    public boolean flywheelIsAtTargetVel() {
        return flyPID.atSetpoint();
    }

    @Override
    public void periodic() {
        runFlywheel();
    }
}
