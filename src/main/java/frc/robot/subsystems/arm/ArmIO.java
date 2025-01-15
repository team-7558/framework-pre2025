package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double elbow_pos_deg = 0.0;
    public double elbow_vel_degps = 0.0;
    public double elbow_volts_V = 0.0;
    public double[] elbow_current_A = new double[] {};

    public double shoulder_pos_deg = 0.0;
    public double shoulder_vel_degps = 0.0;
    public double shoulder_volts_V = 0.0;
    public double[] shoulder_current_A = new double[] {};
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void goToElbowAngle(double pos_deg, boolean first_time) {}

  public default void stopElbow() {}

  public default void goToShoulderAngle(double pos_deg, boolean first_time) {}

  public default void stopShoulder() {}

  public default void toggleBrake() {}
}
