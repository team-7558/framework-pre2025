package frc.robot.subsystems.algaeIntake;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

public class Algae extends StateMachineSubsystemBase<AlgaeStates> {
  private static Algae instance;

  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)
  boolean running = false;

  public Algae(AlgaeIO io) {
    super("Algae");
    this.io = io;
    queueState(AlgaeStates.IDLE);
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("coral", inputs);
  }

  public static Algae getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Algae(new AlgaeIOSim());
          break;
        case REAL:
          instance = new Algae(new AlgaeIOTalonFX());
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
