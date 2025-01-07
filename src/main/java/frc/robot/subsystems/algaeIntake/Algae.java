package frc.robot.subsystems.algaeIntake;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends StateMachineSubsystemBase<AlgaeStates> {
  public final AlgaeIO io;
  public final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  public final Algae2d mech;
  public static Algae instance;
  public boolean beamBroken1;
  public int index;

  public Algae(AlgaeIO io) {
    super("Algae Intake");
    this.io = io;
    mech = Algae2d.getInstance();
    queueState(AlgaeStates.DISABLED);
    index = 0;
    beamBroken1 = inputs.beamBroken1;
  }

  public static Algae getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
        case SIM:
          instance = new Algae(new AlgaeIOSim());
          break;
        case REPLAY:
          break;
        default:
          break;
      }
    }
    return instance;
  }

  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        if (stateInit()) {
          io.stop(); // First time init stuff per entry of state
        }
        break;
      case IDLE:
        io.stop();
        break;
      case INTAKING:
        io.setVoltage(5);
        if (beamBroken1) {
          queueState(AlgaeStates.IDLE);
        }
        break;
      case SPITTING:
        io.setVoltage(-3);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }
}
