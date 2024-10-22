package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class SS {

  public enum State {
    DISABLED,
    IDLE,
    BOOT
  }

  private static SS instance;

  public static SS getInstance() {
    return instance == null ? instance = new SS() : instance;
  }

  private Timer timer;

  private Drive drive;

  private State lastState;
  private State currState;
  private State nextState;

  private SS() {
    lastState = State.DISABLED;
    currState = State.DISABLED;
    nextState = State.DISABLED;

    timer = new Timer();

    drive = Drive.getInstance();
  }

  public void queueState(State s) {
    if (currState != s) {
      nextState = s;
    }
  }

  private void logStates() {
    Logger.recordOutput("SS/lastState", lastState);
    Logger.recordOutput("SS/currState", lastState);
    Logger.recordOutput("SS/nextState", lastState);
    Logger.recordOutput("SS/stateTime", timer.get());
  }

  public void periodic() {
    boolean first = currState != lastState;
    lastState = currState;
    if (currState != nextState) {
      timer.restart();
      currState = nextState;
    }

    switch (currState) {
      case DISABLED:
        if (first) {
          System.out.println("yes?");
          drive.setCurrentState(drive.DISABLED);
        }
      case BOOT:
        if (first) {
          // dw about this for now
          drive.setCurrentState(drive.DISABLED);
        }
        break;
      case IDLE:
        if (first) {
          drive.setCurrentState(drive.STRAFE_N_TURN);
        }
      default:
        break;
    }
  }
}
