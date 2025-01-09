package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private final AltTimer timer = new AltTimer();

  DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);

  private double motor_applied_volts = 0.0;
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          45,
          3.67,
          0.5,
          Units.degreesToRadians(0),
          Units.degreesToRadians(90),
          false,
          Units.degreesToRadians(0)); // Custom arm motor simulation
  private final PIDController armPositionPID = new PIDController(70, 1, 4);
  private double arm_applied_volts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update elevator simulation
    armSim.update(0.02); // 20 ms update
    inputs.slap_pos_deg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.slap_velDegPS = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    inputs.slap_volts = arm_applied_volts;
    inputs.slap_currents =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents

    motorSim.update(0.02); // 20 ms update
    inputs.VelocityDegPS = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
    inputs.AppliedVolts = motor_applied_volts;
    inputs.current_Amps =
        new double[] {
          motorSim.getCurrentDrawAmps(), motorSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setArmVoltage(double left_volts) {
    arm_applied_volts = left_volts;
    armSim.setInputVoltage(left_volts);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor_applied_volts = volts;
    motorSim.setInputVoltage(volts);
  }

  @Override
  public void goToAngle(double degrees, IntakeIOInputs inputs, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      armGoal = new TrapezoidProfile.State(degrees, 0);
      armStartPoint = new TrapezoidProfile.State(inputs.slap_pos_deg, inputs.slap_velDegPS);
    }

    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(armSetpoint.position);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(armSim.getAngleRads());
    setArmVoltage(calculatedVoltage);

    Logger.recordOutput("Intake/ArmSetpoint", armSetpoint.position);
  }

  @Override
  public void stopArm() {
    setArmVoltage(0.0);
  }

  @Override
  public void stopIntake() {
    setIntakeVoltage(0.0);
  }
}
