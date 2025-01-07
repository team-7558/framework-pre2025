package frc.robot.subsystems.coral;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Coral extends StateMachineSubsystemBase<CoralStates> {

  private static Coral instance;

  private final CoralIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)
  boolean running = false;

  public Coral(CoralIO io) {
    super("Coral");
    this.io = io;
    queueState(CoralStates.IDLE);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("coral", inputs);
  }

  public static Coral getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Coral(new CoralIOSim());
          break;
        case REAL:
          instance = new Coral(new CoralIOTalonFX());
          break;
        default:
          break;
      }
    }
    return instance;
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        running = false;
        setVelocity(0);
        break;
      case INTAKING:
        running = true;
        setVelocity(0.5);
        break;
      case SPITTING:
        running = true;
        setVelocity(-0.5);
        break;
      default:
        break;
    }
  }

  @Override
  protected void outputPeriodic() {
    Logger.recordOutput("coral/running", running);
  }

  public void setVelocity(double velocity_DPS) {
    setVoltage(kV * Units.degreesToRadians(velocity_DPS) + kS);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
