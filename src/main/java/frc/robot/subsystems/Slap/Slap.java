package frc.robot.subsystems.Slap;


import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Slap extends StateMachineSubsystemBase<SlapStates> {

  private static Slap instance;

  private final SlapIO io;
  private final Slap2d mech = new Slap2d("ArmActual", new Color8Bit(100, 0, 0));
  private final SlapIOInputsAutoLogged inputs = new SlapIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

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
        if (Constants.currentMode == Mode.SIM) {
          if (Math.abs(inputs.pos_deg - targetAngleDegrees) < 0.5) {
              queueState(SlapStates.HOLDING);
            } else {
              if (stateInit()) {
                first_time = true;
              } else {
                first_time = false;
              }
              io.goToAngle(targetAngleDegrees, inputs, first_time);
            }
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

    mech.setAngle(inputs.pos_deg);
    mech.periodic();


    Logger.recordOutput("Slap/TargetAngleDegrees", targetAngleDegrees);
  }



  public void set(double angle) {
    setTargetAngle(angle);
  }

  public void setTargetAngle(double targetAngleDegrees) {
    this.targetAngleDegrees = targetAngleDegrees;
  }
}
