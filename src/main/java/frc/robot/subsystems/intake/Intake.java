package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachineSubsystemBase<IntakeStates> {

  private static Intake instance;

  private final IntakeIO io;
  private final Intake2d mech = new Intake2d("ArmActual", new Color8Bit(100, 0, 0));
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

  boolean running = false;

  private Intake(IntakeIO io) {
    super("Intake");
    this.io = io;
    setTargetAngle(0);
    queueState(IntakeStates.IDLE);
  }

  public static Intake getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Intake(new IntakeIOSim());
          break;
        case REAL:
          instance = new Intake(new IntakeIOTalonFX());
        default:
          break;
      }
    }
    return instance;
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        break;
      case TRAVELLING:
        if (Math.abs(inputs.slap_pos_deg - targetAngleDegrees) < 0.5) {
          queueState(IntakeStates.HOLDING);
        } else {
          if (stateInit()) {
            first_time = true;
          } else {
            first_time = false;
          }
          io.goToAngle(targetAngleDegrees, inputs, first_time);
        }
        break;
      case HOLDING:
        io.setArmVoltage(0);
        break;
      case INTAKING:
        running = true;
        setIntakeVoltage(2);
      case SPITTING:
        running = false;
        setIntakeVoltage(-2);
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    System.out.println("Before");

    mech.setAngle(inputs.slap_pos_deg);
    mech.periodic();

    Logger.recordOutput("Slap/TargetAngleDegrees", targetAngleDegrees);
    Logger.recordOutput("coral/running", running);
  }

  public void set(double angle) {
    setTargetAngle(angle);
  }

  public void setIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
