package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
  @AutoLog
  public static class HandIOInputs {
    public double intakeVelocityDegPS = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double[] intake_current_Amps = new double[] {};
    public double scoringVelocityDegPS = 0.0;
    public double scoringAppliedVolts = 0.0;
    public double[] scoring_current_Amps = new double[] {};
    public boolean beamBreakActivated = false;
  }

  public default void updateInputs(HandIOInputs inputs) {}

  public default void intakeSetVoltage(double volts) {}

  public default void intakeStop() {}

  public default void scoringSetVoltage(double volts) {}

  public default void scoringStop() {}

  public default void toggleBrake() {}
}
