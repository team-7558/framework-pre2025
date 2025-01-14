package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double elbow_pos_deg = 0.0;
    public double elbow_velDegPS = 0.0;
    public double elbow_volts = 0.0;
    public double[] elbow_currents = new double[] {};

    public double shoulder_pos_deg = 0.0;
    public double shoulder_velDegPS = 0.0;
    public double shoulder_left_volts = 0.0;
    public double[] shoulder_currents = new double[] {};
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setElbowVoltage(double volts) {}

  public default void goToElbowAngle(double pos_deg, ArmIOInputs inputs, boolean first_time) {}

  public default void stopElbow() {}

  public default void toggleBrake() {}
}
