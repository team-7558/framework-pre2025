package frc.robot.subsystems.pinkarm;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawInputs {
    public boolean gamepiece = false;
  }

  public default void updateInputs(ClawInputs inputs) {}

  public default void open(boolean open_or_close) {}
}
