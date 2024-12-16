package frc.robot.subsystems.pinkarm;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawInputs {
    boolean gamepiece = false;
    boolean open = true;
  }
}
