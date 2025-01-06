package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double position_m = 0;
    public double velocity_mps = 0;
    public double volts_V = 0;
    public double[] current_A = new double[] {};

    public boolean hallEffectHit = true;
  }

  public default void updateInputs(ElevatorIOInputs Inputs) {}

  public default void setVoltage(double volts_V) {}

  public default void setVelocity(double vel_mps) {}

  /*
  public default void holdPos(double pos_m) {}

  public default void configurePID(double kP, double kI, double kD) {}

  */
  public default void travelToPos(double pos_m) {}

  public default void resetPos(double pos_m) {}

  public default void stop() {}

  public default void toggleBrake() {}

  public default void zero() {}
}
