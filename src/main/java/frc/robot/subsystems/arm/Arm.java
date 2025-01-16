package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends StateMachineSubsystemBase<ArmStates> {

  // Positive Voltage is intaking
  // Negative Voltage is spitting

  // Positive Voltage is clockwise arm
  // Positive Voltage is counterclockwise arm

  private static Arm instance;

  private final ArmIO io;
  private final Arm2d mech = new Arm2d("ArmActual", new Color8Bit(100, 0, 0));
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)

  private Arm(ArmIO io) {
    super("arm");
    this.io = io;
    setElbowTargetAngle(0);
    queueState(ArmStates.IDLE);
  }

  public static Arm getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Arm(new ArmIOSim());
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
    Logger.processInputs("arm", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        io.stopElbow();
        break;
      case TRAVELLING:
        if (Math.abs(inputs.elbow_pos_deg - targetAngleDegrees) < 0.5) {
          queueState(ArmStates.HOLDING);
        } else {
          if (stateInit()) {
            first_time = true;
          } else {
            first_time = false;
          }
          io.goToElbowAngle(targetAngleDegrees, first_time);
        }
        break;
      case HOLDING:
        io.stopElbow();
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {

    mech.setElbowAngle(inputs.elbow_pos_deg);

    mech.periodic();

    Logger.recordOutput("Arm/TargetAngleDegrees", targetAngleDegrees);
  }

  public void setElbowTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
