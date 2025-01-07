package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  public static class CoralIOInputs {
    public double VelocityDegPS = 0.0;
    public double AppliedVolts = 0.0;
    public double[] current_Amps = new double[] {};
    public boolean beamBreakActivated = false;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
