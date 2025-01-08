package frc.robot.subsystems.claw;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Claw extends StateMachineSubsystemBase<ClawStates> {

  private static Claw instance;

  private final ClawIO io;
  private final Claw2d mech = new Claw2d("ArmActual", new Color8Bit(100, 0, 0));
  private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)

  private Claw(ClawIO io) {
    super("Claw");
    this.io = io;
    setTargetAngle(0);
    queueState(ClawStates.IDLE);
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
    Logger.processInputs("claw", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        setClawVoltage(0.0);
        setArmVoltage(0.0);
        break;
      case TRAVELLING:
        if (Math.abs(inputs.arm_pos_deg - targetAngleDegrees) < 0.5) {
          queueState(ClawStates.HOLDING);
        } else {
          if (stateInit()) {
            first_time = true;
          } else {
            first_time = false;
          }
          io.goToAngle(targetAngleDegrees, first_time);
        }
        break;
      case HOLDING:
        io.setArmVoltage(0);
        break;
      case OPEN:
        setClawVelocity(10);
        break;
      case CLOSE:
        setClawVelocity(-10);
        break;
      case ZEROING:
        queueState(ClawStates.TRAVELLING);
        setTargetAngle(inputs.arm_absolute_pos_deg);
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    if (Math.abs(inputs.claw_volts_V) > 0) {
      mech.Open(true);
    } else {
      System.out.println("IN");
      mech.Open(false);
    }

    mech.setAngle(inputs.arm_pos_deg);

    mech.periodic();

    Logger.recordOutput("Claw/TargetAngleDegrees", targetAngleDegrees);
  }

  public void setAngle(double angle) {
    setTargetAngle(angle);
  }

  public void setArmVoltage(double volts) {
    io.setArmVoltage(volts);
  }

  public void setClawVelocity(double velocity_DPS) {
    setClawVoltage(kV * Units.degreesToRadians(velocity_DPS) + kS);
  }

  public void setClawVoltage(double volts) {
    io.setClawVoltage(volts);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
