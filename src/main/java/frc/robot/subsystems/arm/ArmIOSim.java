package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;
  private ArmIOInputs inputs;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private final AltTimer timer = new AltTimer();

  private final PIDController armPositionPID = new PIDController(15, 1, 4);

  DCMotorSim clawSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          45,
          3.67,
          0.5,
          Units.degreesToRadians(0),
          Units.degreesToRadians(360),
          false,
          Units.degreesToRadians(0)); // Custom arm motor simulation

  private double arm_applied_volts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update elevator simulation

    armSim.update(0.02); // 20 ms update
    inputs.elbow_pos_deg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.elbow_vel_degps = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    inputs.elbow_volts_V = arm_applied_volts;
    inputs.elbow_current_A =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setElbowVoltage(double volts) {
    arm_applied_volts = volts;
    armSim.setInputVoltage(volts);
  }

  @Override
  public void goToElbowAngle(double degrees, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      armGoal = new TrapezoidProfile.State(degrees, 0);
      armStartPoint = new TrapezoidProfile.State(inputs.elbow_pos_deg, inputs.elbow_vel_degps);
    }

    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(armSetpoint.position);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(armSim.getAngleRads());
    setElbowVoltage(calculatedVoltage);

    Logger.recordOutput("Elbow/ArmSetpoint", armSetpoint.position);
  }

  @Override
  public void stopElbow() {
    setElbowVoltage(0.0);
  }
}
