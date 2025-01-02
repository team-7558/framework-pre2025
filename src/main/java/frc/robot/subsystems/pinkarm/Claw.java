package frc.robot.subsystems.pinkarm;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends StateMachineSubsystemBase<ClawModes> {

  private final ClawIO io;
  private static Claw instance;
  private final ClawInputsAutoLogged inputs = new ClawInputsAutoLogged();
  private boolean OPEN_OR_CLOSE;

  private Claw(ClawIO io) {

    super("claw");
    System.out.println("here");
    this.io = io;
    queueState(ClawModes.IDLE);
  }

  public static Claw getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Claw(new ClawIOSim());
          break;
        case REAL:
        default:
          break;
      }
    }
    return instance;
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Claw", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        break;
      case OPEN:
        set(true);
        break;
      case CLOSED:
        set(false);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    io.open(OPEN_OR_CLOSE);
    Logger.recordOutput("claw/open_or_not", OPEN_OR_CLOSE);
    Logger.recordOutput("claw/opened", true);
  }

  public void set(boolean open) {
    OPEN_OR_CLOSE = open;
  }
}
