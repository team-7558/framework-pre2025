package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
  @AutoLog
  public static class HandIOInputs {
    public double VelocityDegPS = 0.0;
    public double AppliedVolts = 0.0;
    public double[] current_Amps = new double[] {};
    public boolean beamBreakActivated = false;
  }

  public default void updateInputs(HandIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
