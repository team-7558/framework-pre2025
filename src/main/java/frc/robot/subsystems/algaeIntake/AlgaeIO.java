package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;



public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double VelocityDegPS = 0.0;
    public double AppliedVolts = 0.0;
    public double[] current_Amps = new double[] {};
    public boolean beamBreakActivated = false;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
