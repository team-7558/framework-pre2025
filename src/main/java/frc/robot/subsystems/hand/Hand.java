package frc.robot.subsystems.hand;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hand extends StateMachineSubsystemBase<HandStates> {
  private static Hand instance;
  private Hand2d mech;
  public final HandIO io;
  public final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();
  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)

  public Hand(HandIO io) {
    super("Hand");
    this.io = io;
    queueState(HandStates.IDLE);
    mech = new Hand2d();
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("hand", inputs);
  }

  public static Hand getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Hand(new HandIOSim());
          break;
        case REAL:
          instance = new Hand(new HandIOTalonFX());
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
        setVoltage(0.0);
        break;
      case INTAKING:
        setVoltage(3.0);
        if (inputs.beamBreakActivated) {
          queueState(HandStates.IDLE);
        }
        break;
      case SPITTING:
        setVoltage(-3.0);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    mech.setAngle(inputs.VelocityDegPS);
    mech.periodic();
  }

  public void setVelocity(double velocity_DPS) {
    setVoltage(kV * Units.degreesToRadians(velocity_DPS) + kS);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
