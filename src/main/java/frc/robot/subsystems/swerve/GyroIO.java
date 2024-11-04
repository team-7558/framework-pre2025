package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw_Rot2d = new Rotation2d();
    public Rotation2d pitch_Rot2d = new Rotation2d();
    public Rotation2d roll_Rot2d = new Rotation2d();
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public double yawVel_radps = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void zero() {}
}
