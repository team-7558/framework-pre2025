package frc.robot.subsystems.drive;

import frc.robot.util.OdometryTimeStampsSim;
import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation sim;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.sim = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.odometryYawPositions = sim.getCachedGyroReadings();
    inputs.odometryYawTimestamps = OdometryTimeStampsSim.getTimeStamps();
    inputs.yaw_Rot2d = sim.getGyroReading();
    inputs.yawVel_radps = sim.getMeasuredAngularVelocityRadPerSec();
  }

  @Override
  public void zero() {
    sim.setRotation(new Rotation2d());
  }
}
