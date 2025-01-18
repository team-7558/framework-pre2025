package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
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
  public boolean Elbow_first_time = true;
  public boolean Shoulder_first_time = true;

  private double ElbowTargetAngleDegrees;
  private double ShoulderTargetAngleDegrees;

  private double previousElbowTargetAngleDegrees;
  private double previousShoulderTargetAngleDegrees;

  private double ElbowForceGravity;
  private double ShoulderForceGravity;



  private double ARMFORCE = 10*9.8*0.01;


  double kV = 0.5; // Volts per rad/s (example value)
  double kS = 1.0; // Static friction voltage (example value)

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
        io.stopElbow();
        break;
      case TRAVELLING:
        if (Math.abs(inputs.elbow_pos_deg - ElbowTargetAngleDegrees) < 0.5) {
          queueState(ArmStates.HOLDING);
        } else {
          if (this.ElbowTargetAngleDegrees != this.previousElbowTargetAngleDegrees) {
            Elbow_first_time = true;
          } else {
            Elbow_first_time = false;
          }

          if (this.ShoulderTargetAngleDegrees != this.previousShoulderTargetAngleDegrees) {
            Shoulder_first_time = true;
          } else {
            Shoulder_first_time = false;
          }
          io.goToElbowAngle(ElbowTargetAngleDegrees, inputs, Elbow_first_time, calcElbowFeedForwardVoltage(ShoulderTargetAngleDegrees, ElbowTargetAngleDegrees));
          io.goToShoulderAngle(ShoulderTargetAngleDegrees, inputs, Shoulder_first_time, calcShoulderFeedForwardVoltage(ShoulderTargetAngleDegrees, ElbowTargetAngleDegrees));
        }
        break;
      case HOLDING:
        io.stopElbow();
        io.stopShoulder();
        break;
      default:
        break;
    }
  }

  @Override
  public void outputPeriodic() {

    mech.setElbowAngle(inputs.elbow_pos_deg);
    mech.setShoulderAngle(inputs.shoulder_pos_deg);
    mech.periodic();

    Logger.recordOutput("Arm/Elbow/TargetAngleDegrees", ElbowTargetAngleDegrees);
    Logger.recordOutput("Arm/Shoulder/TargetAngleDegrees", ShoulderTargetAngleDegrees);
    Logger.recordOutput("Arm/Elbow/FeedForwardVoltage", ElbowForceGravity);
    Logger.recordOutput("Arm/Shoulder/FeedForwardVoltage", ShoulderForceGravity);
  }

  public void setElbowTargetAngle(double targetAngleDegrees) {
    previousElbowTargetAngleDegrees = this.ElbowTargetAngleDegrees;
    this.ElbowTargetAngleDegrees = targetAngleDegrees;
  }

  public void setShoulderTargetAngle(double targetAngleDegrees) {
    previousShoulderTargetAngleDegrees = this.ShoulderTargetAngleDegrees;
    this.ShoulderTargetAngleDegrees = targetAngleDegrees;
  }

  public double calcShoulderFeedForwardVoltage(double shoulder_pos_deg, double elbow_pos_deg) {
    ShoulderForceGravity = ARMFORCE*Math.cos(Units.degreesToRadians(shoulder_pos_deg))*Math.sin(Units.degreesToRadians(elbow_pos_deg));
    return ShoulderForceGravity;
  }

  public double calcElbowFeedForwardVoltage(double shoulder_pos_deg, double elbow_pos_deg) {
    ElbowForceGravity = ARMFORCE*Math.cos(Units.degreesToRadians(elbow_pos_deg))*Math.sin(Units.degreesToRadians(shoulder_pos_deg));
    return ElbowForceGravity;
  }
}
