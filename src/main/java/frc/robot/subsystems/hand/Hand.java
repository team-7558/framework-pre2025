package frc.robot.subsystems.hand;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hand extends StateMachineSubsystemBase<HandStates> {
  public double scoreVolts;
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
    scoreVolts = 0.0;
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
        intakeSetVoltage(0.0);
        scoringSetVoltage(0.0);
        break;
      case INTAKING:
        intakeSetVoltage(3.0);
        if (inputs.beamBreakActivated) {
          intakeSetVoltage(0.0);
        }
        break;
      case SPITTING:
        scoringSetVoltage(scoreVolts);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    mech.setAngle(inputs.intakeVelocityDegPS);
    mech.score(inputs.scoringVelocityDegPS);
    mech.periodic();
  }

  public void scoringSetVoltage(double volts) {
    io.scoringSetVoltage(volts);
  }

  public void intakeSetVelocity(double velocity_DPS) {
    intakeSetVoltage(kV * Units.degreesToRadians(velocity_DPS) + kS);
  }

  public void intakeSetVoltage(double volts) {
    io.intakeSetVoltage(volts);
  }
}
