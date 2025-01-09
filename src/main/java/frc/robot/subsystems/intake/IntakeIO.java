package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double slap_pos_deg = 0.0;
    public double slap_velDegPS = 0.0;
    public double slap_volts = 0.0;
    public double[] slap_currents = new double[] {};

    public double VelocityDegPS = 0.0;
    public double AppliedVolts = 0.0;
    public double[] current_Amps = new double[] {};
    public boolean beamBreakActivated = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setArmVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default void goToAngle(double pos_deg, IntakeIOInputs inputs, boolean first_time) {}

  public default void stopIntake() {}

  public default void stopArm() {}

  public default void toggleBrake() {}
}
