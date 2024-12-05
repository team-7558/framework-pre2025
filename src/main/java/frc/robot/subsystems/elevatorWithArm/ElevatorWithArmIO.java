package frc.robot.subsystems.elevatorWithArm;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorWithArmIO {
  @AutoLog
  public static class ElevatorWithArmIOInputs {
    public double elevPositionM = 0.0;
    public double armPositionM = 0.0;
    public double velocityMPerSecond = 0.0;
    public double velocityDegPerSecond = 0.0;
    public double appliedVoltsElev = 0.0;
    public double appliedVoltsArm = 0.0;
    public double[] currentAmpsElev = new double[] {};
    public double[] currentAmpsArm = new double[] {};
  }

  public default void updateInputs(ElevatorWithArmIOInputs inputs) {}

  public default void setVoltageElev(double voltage_V) {}

  public default void setVoltageArm(double voltage_V) {}

  public default void TravelPosElev(double elevPositionM) {}

  public default void TravelAngleArm(double armAngleM) {}

  public default void holdPosElev(double pos_m) {}

  public default void holdPosArm(double angleRad) {}

  public default void stop() {}
}