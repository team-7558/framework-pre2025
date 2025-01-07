package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double algaeVolts;
    public double algaeSpeed;
    public double[] algaeAmps;
    public double simOnlyRot;
    public boolean beamBroken1;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void breakBeamManual(boolean trueOrFalse) {}

  public default void stop() {}
}
