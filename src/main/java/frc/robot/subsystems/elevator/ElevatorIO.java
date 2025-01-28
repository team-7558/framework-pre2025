package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double pos_m = 0.0;
    public double vel_mps = 0.0;
    public double volts_V = 0.0;
    public double[] currents_A = new double[] {};

    public boolean hallEffect = false;
  }

  public default void updateInputs(ElevatorIOInputs Inputs) {}

  public default void setVoltage(double volts_V) {}

  public default void setVel(double vel_mps) {}

  public default void holdPos(double pos_m) {}

  public default void travelToPos(double pos_m) {}

  public default void resetPos(double pos_m) {}

  public default void stop() {}

  public default void setBrake() {}
}
