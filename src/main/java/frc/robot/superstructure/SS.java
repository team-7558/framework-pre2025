package frc.robot.superstructure;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.util.AltTimer;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

public class SS implements IStateMachine<InternalState> {

  private static SS instance;

  public static SS getInstance() {
    return instance == null ? instance = new SS() : instance;
  }

  private Drive drive;

  private AltTimer timer;
  private Intention intention;
  private InternalState state;
  private boolean firstStep;

  private boolean booted;

  private SS() {
    intention = Intention.IDLE;
    state = InternalState.IDLE;

    timer = new AltTimer();
    firstStep = false;
    booted = false;

    drive = Drive.getInstance();
  }

  @Override
  public void handleStateMachine() {
    if (!booted && !isState(InternalState.DISABLED)) {
      queueState(InternalState.BOOT);
    }

    switch (state) {
      case DISABLED:
        drive.queueState(PathingMode.DISABLED);
        break;
      case IDLE:
        drive.queueState(PathingMode.FIELD_RELATIVE);
        break;
      case BOOT:
        drive.queueState(PathingMode.DISABLED);

        if (timer.after(0.1)) {
          booted = true;
          queueState(InternalState.IDLE);
        }
        break;
      default:
        break;
    }

    logStates();
  }

  private void logStates() {
    Logger.recordOutput("SS/Booted?", booted);
    Logger.recordOutput("SS/Intention", intention);
    Logger.recordOutput("SS/State", state);
    Logger.recordOutput("SS/StateTimer", timer.time());
  }

  public void intend(Intention intention) {
    this.intention = intention;
  }

  @Override
  public InternalState getState() {
    return state;
  }

  @Override
  public void queueState(InternalState nextState) {
    if (!state.equals(nextState)) {
      state = nextState;
      timer.reset();
      firstStep = true;
    } else {
      firstStep = false;
    }
  }

  @Override
  public boolean stateInit() {
    if (firstStep) {
      firstStep = false;
      return true;
    }
    return false;
  }

  @Override
  public boolean isState(InternalState state) {
    return this.state.equals(state);
  }
}
