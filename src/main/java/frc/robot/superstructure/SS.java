package frc.robot.superstructure;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.util.AltTimer;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

public class SS implements IStateMachine<InternalState> {

  private static SS instance;

  public static SS getInstance() {
    return instance == null ? instance = new SS() : instance;
  }

  private Drive drive;
  private Arm arm;
  private Elevator elevator;

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
    arm = Arm.getInstance();
    elevator = Elevator.getInstance();
  }

  @Override
  public void handleStateMachine() {
    if (!booted && !isState(InternalState.DISABLED)) {
      queueState(InternalState.BOOT);
    }

    switch (state) {
      case DISABLED:
        drive.queueState(PathingMode.DISABLED);
        elevator.queueState(ElevatorState.DISABLED);
        break;
      case IDLE:
        arm.setArmTarget(15);
        arm.queueState(ArmState.HOLDING_PIECE);
        drive.queueState(PathingMode.FIELD_RELATIVE);
        break;
      case BOOT:
        drive.queueState(PathingMode.DISABLED);
        elevator.queueState(ElevatorState.ZEROING);
        arm.queueState(ArmState.ZEROING);
        if (elevator.getZeroed()) {
          arm.queueState(ArmState.ZEROING);
          arm.setArmTarget(15);
          arm.queueState(ArmState.IDLE);
          booted = true;
          queueState(InternalState.IDLE);
        }
        break;
      case SCORING_UP:
        elevator.set(Elevator.ELEV_SCORING_TOP);
        arm.queueState(ArmState.HOLDING_PIECE);
        elevator.queueState(ElevatorState.HOLDING);
        if (elevator.getHeight() >= Elevator.ELEV_SCORING_TOP - 1) {
          // System.out.println("at target!!!!!!!!!!!");
          arm.setArmTarget(30);
        } else {
          arm.setArmTarget(15);
        }
        break;
      case SCORING_DOWN:
        if (elevator.getHeight() >= Elevator.ELEV_SCORING_DOWN) {
          arm.setArmTarget(20, 0.5);
          arm.queueState(ArmState.HOLDING_PIECE);
        } else if (elevator.getHeight() <= Elevator.ELEV_SCORING_DOWN
            && elevator.getHeight() >= Elevator.ELEV_SCORING_DOWN - 0.4) {
          arm.queueState(ArmState.SPITTING);
          arm.setArmTarget(30, 0.5);
        } else if (elevator.getHeight() < Elevator.ELEV_SCORING_DOWN - 0.7) {
          arm.setArmTarget(15, 0.3);
          arm.queueState(ArmState.HOLDING_PIECE);
        }
        elevator.set(0.2);
        elevator.queueState(ElevatorState.HOLDING);
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
