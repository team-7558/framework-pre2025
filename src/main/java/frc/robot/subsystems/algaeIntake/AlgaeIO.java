package frc.robot.subsystems.algaeIntake;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    public double algaeVolts;
    public double[] algaeAmps;
    public double simOnlyRot;
    public boolean beamBroken1;
    public boolean beamBroken2;
  }

  public default void updateInputs(AlgaeIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void breakBeamManual(boolean trueOrFalse, int bb) {}

  public default void stop() {}
}
