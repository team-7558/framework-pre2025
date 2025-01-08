package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.Slap.SlapIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Claw extends StateMachineSubsystemBase<ClawStates>{

  private static Claw instance;

  private final ClawIO io;
  private final Claw2d mech = new Claw2d("ArmActual", new Color8Bit(100, 0, 0));
  private final ClawIOInputsAutoLogged inputs = new SlapIOInputsAutoLogged();
  public boolean first_time = true;

  private double targetAngleDegrees;

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
