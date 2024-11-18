package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OdometryTimeStampsSim;
import org.ironmaple.simulation.drivesims.GyroSimulation;

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
    inputs.pitch_Rot2d = new Rotation2d();
    inputs.roll_Rot2d = new Rotation2d();
    inputs.yawVel_radps = sim.getMeasuredAngularVelocityRadPerSec();
  }

  public void zero() {
    sim.setRotation(new Rotation2d());
  }
}
