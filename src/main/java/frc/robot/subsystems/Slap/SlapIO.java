package frc.robot.subsystems.Slap;

import org.littletonrobotics.junction.AutoLog;

public interface SlapIO {
  @AutoLog
  public static class SlapIOInputs {
    public double pos_deg = 0.0;
    public double velDegPS = 0.0;
    public double volts = 0.0;
    public double[] currents = new double[] {};
  }

  public default void updateInputs(SlapIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void goToAngle(double pos_deg) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
