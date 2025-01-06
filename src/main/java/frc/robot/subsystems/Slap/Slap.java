package frc.robot.subsystems.Slap;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class Slap extends StateMachineSubsystemBase<SlapStates> {

  private static Slap instance;

  private final SlapIO io;
  private final Slap2d mech = new Slap2d("ArmActual", new Color8Bit(100, 0, 0));
  private final SlapIOInputsAutoLogged inputs = new SlapIOInputsAutoLogged();
  private final AltTimer timer = new AltTimer();

  private double targetAngleDegrees;

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private Slap(SlapIO io) {
    super("Slap");
    this.io = io;
    setTargetAngle(0);
    queueState(SlapStates.IDLE);
  }

  public static Slap getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Slap(new SlapIOSim());
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
    Logger.processInputs("slap", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        break;
      case TRAVELLING:
        if (Math.abs(inputs.pos_deg - targetAngleDegrees) < 0.5) {
          queueState(SlapStates.HOLDING);
        } else {
          if (stateInit()) {
            System.out.println("Travelling once");
            timer.reset();

            armGoal = new TrapezoidProfile.State(targetAngleDegrees, 0);
            armStartPoint = new TrapezoidProfile.State(inputs.pos_deg, inputs.velDegPS);
          }
          armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
        }
        break;
      case HOLDING:
        io.setVoltage(0);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    System.out.println("Before");
    if (armSetpoint != null) {
      System.out.println("in");
      io.goToAngle(armSetpoint.position);
      System.out.println("out");
    }
    mech.setAngle(inputs.pos_deg);

    mech.periodic();

    Logger.recordOutput("Slap/TargetAngleDegrees", targetAngleDegrees);
    Logger.recordOutput("Slap/ArmSetpoint", armSetpoint.position);
  }

  public void PlaceEndEffector(double x, double y) {
    double magnitude = Math.sqrt(0.1 * x * 0.1 * x + 0.1 * y * 0.1 * y);
    double angle = Units.radiansToDegrees(Math.atan2(0.1 * y, 0.1 * x));

    set(magnitude, angle);
  }

  public void set(double meters, double angle) {
    setTargetAngle(angle);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
