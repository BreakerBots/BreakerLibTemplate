// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.Devices.BreakerPigeon2;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private BreakerPigeon2 imu;
  int cycles;

  public ExampleSubsystem() {
    imu = new BreakerPigeon2(5, true);
    imu.resetGlobalPosition();
  }

  @Override
  public void periodic() {
    cycles ++;
    // This method will be called once per scheduler run
    String outString = String.format( " X: %.3f Y: %.3f ACCEL X: %.3f ACCEL Y: %.3f BIAS Y: %.3f BIAS X: %.3f\n",
        imu.getGlobalPositionComponents(1), imu.getGlobalPositionComponents(2),
        imu.getIns2AccelX(), imu.getIns2AccelY(), imu.getAccelYBias(), imu.getAccelXBias());
    System.out.println(outString);
  //  if (cycles % 150 == 0) {
  //    imu.reset();
  //    imu.resetGlobalPosition();
  //  }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
