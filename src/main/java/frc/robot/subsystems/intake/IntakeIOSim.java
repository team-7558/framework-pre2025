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

  public TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(30, 200);

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
          Units.degreesToRadians(90)); // Custom arm motor simulation

  private final PIDController armPositionPID = new PIDController(70, 1, 4);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    armSim.update(0.02); // 20 ms update
    inputs.slap_pos_deg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.slap_velDegPS = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    inputs.slap_volts = applied_volts;
    inputs.slap_currents =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents

    inputs.AppliedVolts = applied_volts;
    inputs.VelocityDegPS = motorSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void goToAngle(double degrees, IntakeIOInputs inputs, boolean first_time) {
    if (first_time) {
      //System.out.println("Travelling once");
      timer.reset();
      armGoal = new TrapezoidProfile.State(degrees, 0);
      armStartPoint = new TrapezoidProfile.State(inputs.slap_pos_deg, inputs.slap_velDegPS);
      //System.out.println(inputs.slap_pos_deg + " " + inputs.slap_velDegPS);
    }

    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(armSetpoint.position);
    armPositionPID.setSetpoint(targetRadians);
    double armCalculatedVoltage = armPositionPID.calculate(armSim.getAngleRads());
    applied_volts = armCalculatedVoltage;
    armSim.setInputVoltage(armCalculatedVoltage);

    Logger.recordOutput("Slap/armSetpoint", armSetpoint.position);
    Logger.recordOutput("Slap/armGoal", armGoal.position);
    Logger.recordOutput("Slap/armGoal", armStartPoint.position);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    motor_applied_volts = volts;
    motorSim.setInputVoltage(volts);
  }

  @Override
  public void stopIntake() {
    setIntakeVoltage(0.0);
  }
}
