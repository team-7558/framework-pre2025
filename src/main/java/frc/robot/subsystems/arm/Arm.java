package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends StateMachineSubsystemBase<ArmStates> {

  private static Arm instance;

  private final ArmIO io;
  private final Arm2d mech = new Arm2d("ArmActual", new Color8Bit(100, 0, 0));
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

  private Arm(ArmIO io) {
    super("Arm");
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
    Logger.processInputs("Arm", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        break;
      case IDLE:
        break;
      case ELBOWTRAVELLING:
        if (Math.abs(inputs.elbow_pos_deg - targetAngleDegrees) < 0.5) {
          queueState(ArmStates.ELBOWHOLDING);
        } else {
          if (stateInit()) {
            first_time = true;
          } else {
            first_time = false;
          }
          io.goToElbowAngle(targetAngleDegrees, inputs, first_time);
        }

        break;
      case ELBOWHOLDING:
        io.setElbowVoltage(0);
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {
    System.out.println("Before");

    mech.setElbowAngle(inputs.elbow_pos_deg);
    mech.periodic();

    Logger.recordOutput("Arm/TargetAngleDegrees", targetAngleDegrees);
  }

  public void set(double angle) {
    setElbowTargetAngle(angle);
  }

  public void setElbowTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
