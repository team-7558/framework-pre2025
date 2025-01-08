package frc.robot.subsystems.claw;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {
    public double claw_velocity_degps = 0.0;
    public double claw_volts_V = 0.0;
    public double[] claw_currents_A = new double[] {};

    public double arm_absolute_pos_deg = 0.0;
    public double arm_pos_deg = 0.0;
    public double arm_velocity_DegPS = 0.0;
    public double arm_volts_V = 0.0;
    public double[] arm_currents_A = new double[] {};
  }

  public default void updateInputs(ClawIOInputs inputs) {}

  public default void setArmVoltage(double volts) {}

  public default void setClawVoltage(double volts) {}

  public default void goToAngle(double degrees, boolean first_time) {}

  public default void stop_arm() {}

  public default void stop_claw() {}
}
