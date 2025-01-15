package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {

  private ArmIOInputs inputs;

  private TrapezoidProfile.State ElbowArmSetpoint;
  private TrapezoidProfile.State ElbowArmStartPoint;
  private TrapezoidProfile.State ElbowArmGoal;

  private final TrapezoidProfile.Constraints ElbowArmConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile ElbowArmProfile = new TrapezoidProfile(ElbowArmConstraints);

  private final AltTimer timer = new AltTimer();

  private TrapezoidProfile.State ShoulderArmSetpoint;
  private TrapezoidProfile.State ShoulderArmStartPoint;
  private TrapezoidProfile.State ShoulderArmGoal;

  private final TrapezoidProfile.Constraints ShoulderArmConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile ShoulderArmProfile = new TrapezoidProfile(ElbowArmConstraints);

  private final AltTimer ShoulderTimer = new AltTimer();

  private final SingleJointedArmSim ElbowArmSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          45,
          3.67,
          0.5,
          Units.degreesToRadians(-110),
          Units.degreesToRadians(110),
          false,
          Units.degreesToRadians(0)); // Custom arm motor simulation

  private final PIDController ElbowArmPositionPID = new PIDController(10, 0, 0);
  private double applied_volts = 0.0;

  private final SingleJointedArmSim ShoulderArmSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          45,
          3.67,
          0.5,
          Units.degreesToRadians(-45),
          Units.degreesToRadians(225),
          false,
          Units.degreesToRadians(0)); // Custom arm motor simulation

  private final PIDController ShoulderArmPositionPID = new PIDController(70, 1, 4);
  private double Shoulder_Applied_volts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update elevator simulation
    ElbowArmSim.update(0.02); // 20 ms update
    inputs.elbow_pos_deg = Units.radiansToDegrees(ElbowArmSim.getAngleRads());
    inputs.elbow_vel_degps = Units.radiansToDegrees(ElbowArmSim.getVelocityRadPerSec());
    inputs.elbow_volts_V = applied_volts;
    inputs.elbow_current_A =
        new double[] {
          ElbowArmSim.getCurrentDrawAmps(), ElbowArmSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents

    ShoulderArmSim.update(0.02); // 20 ms update
    inputs.shoulder_pos_deg = Units.radiansToDegrees(ShoulderArmSim.getAngleRads());
    inputs.shoulder_vel_degps = Units.radiansToDegrees(ShoulderArmSim.getVelocityRadPerSec());
    inputs.shoulder_volts_V = Shoulder_Applied_volts;
    inputs.shoulder_current_A =
        new double[] {
          ShoulderArmSim.getCurrentDrawAmps(), ShoulderArmSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void goToElbowAngle(double degrees, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      ElbowArmGoal = new TrapezoidProfile.State(degrees, 0);
      ElbowArmStartPoint = new TrapezoidProfile.State(inputs.elbow_pos_deg, inputs.elbow_vel_degps);
    }

    ElbowArmSetpoint = ElbowArmProfile.calculate(timer.time(), ElbowArmStartPoint, ElbowArmGoal);
    // Check if the target is valid (optional safety check)
    double ElbowtargetRadians = Units.degreesToRadians(ElbowArmSetpoint.position);
    ElbowArmPositionPID.setSetpoint(ElbowtargetRadians);
    double ElbowcalculatedVoltage = ElbowArmPositionPID.calculate(ElbowArmSim.getAngleRads());
    applied_volts = ElbowcalculatedVoltage;
    ElbowArmSim.setInputVoltage(ElbowcalculatedVoltage);

    Logger.recordOutput("ElbowArm/ArmSetpoint", ElbowArmSetpoint.position);
    Logger.recordOutput("ElbowArm/ElbowArmGoal", ElbowArmGoal.position);
    Logger.recordOutput("ElbowArm/ElbowArmGoal", ElbowArmStartPoint.position);
  }

  @Override
  public void goToShoulderAngle(double degrees, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      ShoulderArmGoal = new TrapezoidProfile.State(degrees, 0);
      ShoulderArmStartPoint =
          new TrapezoidProfile.State(inputs.shoulder_pos_deg, inputs.shoulder_vel_degps);
    }

    ShoulderArmSetpoint =
        ShoulderArmProfile.calculate(timer.time(), ShoulderArmStartPoint, ShoulderArmGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(ShoulderArmSetpoint.position);
    ShoulderArmPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = ShoulderArmPositionPID.calculate(ShoulderArmSim.getAngleRads());
    Shoulder_Applied_volts = calculatedVoltage;
    ShoulderArmSim.setInputVoltage(calculatedVoltage);

    Logger.recordOutput("ArmShoulder/ArmSetpoint", ShoulderArmSetpoint.position);
  }

  @Override
  public void stopElbow() {
    applied_volts = 0;
    ElbowArmSim.setInputVoltage(0);
  }

  @Override
  public void stopShoulder() {
    Shoulder_Applied_volts = 0;
    ShoulderArmSim.setInputVoltage(0);
  }
}
