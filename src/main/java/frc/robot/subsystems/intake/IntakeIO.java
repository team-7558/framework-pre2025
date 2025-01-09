package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
  public static class IntakeIOInputs {
    public double pos_deg = 0.0;
    public double left_velDegPS = 0.0;
    public double left_volts = 0.0;
    public double[] left_currents = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void goToAngle(double pos_deg, IntakeIOInputs inputs, boolean first_time) {}

  public default void stop() {}

  public default void toggleBrake() {}
}
