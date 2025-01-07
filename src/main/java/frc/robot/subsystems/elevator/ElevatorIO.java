package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double posm = 0.0;
    public double velmps = 0.0;
    public double voltsV = 0.0;
    public double[] currentsA = new double[] {};

    public boolean hallEffect = false;
  }

  public default void updateInputs(ElevatorIOInputs Inputs) {}

  public default void setVoltage(double volts_V) {}

  public default void setVel(double vel_mps) {}

  public default void holdPos(double pos_m) {}

  public default void travelToPos(double pos_m) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void resetPos(double pos_m) {}

  public default void stop() {}

  public default void toggleBrake() {}

  public default void zero() {}
}