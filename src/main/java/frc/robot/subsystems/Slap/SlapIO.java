package frc.robot.subsystems.Slap;

import org.littletonrobotics.junction.AutoLog;

public interface SlapIO {
  @AutoLog
  public static class SlapIOInputs {
    public double pos_deg = 0.0;
    public double left_velDegPS = 0.0;
    public double left_volts = 0.0;
    public double[] left_currents = new double[] {};
  }

  public default void updateInputs(SlapIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void goToAngle(double pos_deg, SlapIOInputs inputs, boolean first_time) {}

  public default void stop() {}

  public default void toggleBrake() {}

}
