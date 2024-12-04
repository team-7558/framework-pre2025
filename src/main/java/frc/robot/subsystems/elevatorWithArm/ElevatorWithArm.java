package frc.robot.subsystems.elevatorWithArm;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class ElevatorWithArm extends StateMachineSubsystemBase<ElevatorWithArmStates> {
  private final ElevatorWithArmIO io;
  private final ElevatorWithArmIOInputsAutoLogged inputs = new ElevatorWithArmIOInputsAutoLogged();
  public static ElevatorWithArm instance;
  public double targetHeight_m;
  public double angleRad;

  public ElevatorWithArm(ElevatorWithArmIO io) {
    super("Elevator With Arm");
    this.io = io;
    queueState(ElevatorWithArmStates.DISABLED);
    targetHeight_m = 0.5;
    angleRad = 0;
  }

  public static ElevatorWithArm getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
        case SIM:
          instance = new ElevatorWithArm(new ElevatorWithArmIOSim());
          break;
        case REPLAY:
          break;
        default:
          break;
      }
    }
    return instance;
  }

  public void setVoltageElev(double voltage) {
    io.setVoltageElev(voltage);
  }

  public void setVoltageArm(double voltage) {
    io.setVoltageArm(voltage);
  }

  public void TravelPosElev(double pos) {
    io.TravelPosElev(pos);
  }

  public void TravelArm(double armAngle) {
    io.TravelAngleArm(armAngle);
  }

  public void holdPosElev(double pos_m) {
    io.holdPosElev(pos_m);
  }

  public void holdPosArm(double armAngle) {
    io.holdPosArm(armAngle);
  }

  public boolean atTargetHeight() {
    return atTargetHeight(0.02);
  }

  public boolean atTargetHeight(double tol) {
    return atHeight(targetHeight_m, tol);
  }

  public boolean atHeight(double height_m, double tol) {
    return Util.inRange(height_m - inputs.elevPositionM, tol);
  }

  public boolean atTargetAngle() {
    return atAngleHeight(0.02);
  }

  public boolean atAngleHeight(double tol) {
    return atHeight(angleRad, tol);
  }

  public boolean atAngle(double height_m, double tol) {
    return Util.inRange(height_m - inputs.armPositionM, tol);
  }

  public void stop() {
    io.stop();
  }

  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        if (stateInit()) {
          setVoltageElev(0); // First time init stuff per entry of state
        }
        break;
      case IDLE:
        setVoltageElev(0);
        break;
      case TRAVELLINGELEVATOR:
        if (!atTargetHeight()) {
          TravelPosElev(targetHeight_m);
        } else {
          queueState(ElevatorWithArmStates.HOLDINGELEVATOR);
        } break;
      case HOLDINGELEVATOR:
        if (atTargetHeight()) {
          holdPosElev(targetHeight_m);
        } else {
          queueState(ElevatorWithArmStates.TRAVELLINGELEVATOR);
        }break;
      case TRAVELLINGARM:
        if (!atTargetAngle()) {
          TravelArm(angleRad);
        } else {
          queueState(ElevatorWithArmStates.HOLDINGARM);
        }break;
      case HOLDINGARM:
        if (atTargetAngle()) {
          holdPosElev(angleRad);
        } else {
          queueState(ElevatorWithArmStates.TRAVELLINGARM);
        }break;
      default:
    }
  }

  @Override
  public void outputPeriodic() {

    io.updateInputs(inputs);
    Logger.processInputs("ElevatorWithArm", inputs);
    Logger.recordOutput("TargetDistance", targetHeight_m);
    Logger.recordOutput("TargetAngle",angleRad);
  }
}
