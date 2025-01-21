package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachineSubsystemBase<IntakeStates> {

  private static Intake instance;

  private double previousAngleDegrees;
  private boolean newAngle;
  private final IntakeIO io;
  private final Intake2d mech = new Intake2d("ArmActual", new Color8Bit(100, 0, 0));
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public double dif;

  private double targetAngleDegrees;

  boolean running = false;

  private Intake(IntakeIO io) {
    super("Intake");
    this.io = io;
    queueState(IntakeStates.IDLE);
    newAngle = false;
    inputs.slap_pos_deg = 90;
    previousAngleDegrees = 90;
    setTargetAngle(90);
    io.goToAngle(targetAngleDegrees, inputs, true);
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
    dif = (Math.abs(inputs.slap_pos_deg - targetAngleDegrees));
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        break;
      case TRAVELLING:
        // System.out.println(inputs.slap_pos_deg - targetAngleDegrees);
        io.goToAngle(targetAngleDegrees, inputs, newAngle);
        if (atTargetAngle()) {
          // io.stopArm();
          queueState(IntakeStates.HOLDING);
        }
        break;
      case HOLDING:
        if (!atTargetAngle()) {
          queueState(IntakeStates.TRAVELLING);
        } else io.goToAngle(targetAngleDegrees, inputs, newAngle);
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
    // System.out.println("Before");

    mech.setAngle(inputs.slap_pos_deg);
    mech.periodic();

    Logger.recordOutput("Slap/TargetAngleDegrees", targetAngleDegrees);
    Logger.recordOutput("Slap/CurrentAngle", inputs.slap_pos_deg);
    Logger.recordOutput("coral/running", running);
    Logger.recordOutput("Slap/newAngle?", newAngle);
    Logger.recordOutput("Slap/previous", previousAngleDegrees);
    Logger.recordOutput("Slap/dif", dif);
  }

  public void set(double angle) {
    setTargetAngle(angle);
  }

  public void setIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  public boolean atAngle(double angle, double tol) {
    return Util.inRange(angle - inputs.slap_pos_deg, tol);
  }

  public boolean atTargetAngle() {
    return atTargetAngle(0.5);
  }

  public boolean atTargetAngle(double tol) {
    return atAngle(targetAngleDegrees, tol);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
    if (targetAngleDegrees == previousAngleDegrees) {
      newAngle = false;
    } else {
      newAngle = true;
    }
    previousAngleDegrees = targetAngleDegrees;
  }
}
