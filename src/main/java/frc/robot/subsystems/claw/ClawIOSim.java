package frc.robot.subsystems.claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class ClawIOSim implements ClawIO {

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private final AltTimer timer = new AltTimer();

    private final PIDController armPositionPID = new PIDController(70, 1, 4);



    DCMotorSim clawSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);
    DCMotorSim armSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private double arm_applied_volts = 0.0;
  private double claw_applied_volts = 0.0;

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    // Update elevator simulation
    clawSim.update(0.02); // 20 ms update
    inputs.claw_velocityDegPS = Units.radiansToDegrees(clawSim.getAngularVelocityRadPerSec());
    inputs.claw_volts_V = claw_applied_volts;
    inputs.claw_currents_A =
        new double[] {
          clawSim.getCurrentDrawAmps(), clawSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents

    armSim.update(0.02); // 20 ms update
    inputs.arm_pos_deg = Units.radiansToDegrees(armSim.getAngularPositionRad());
    inputs.arm_velocityDegPS = Units.radiansToDegrees(armSim.getAngularVelocityRadPerSec());
    inputs.arm_volts_V = arm_applied_volts;
    inputs.arm_currents_A =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setClawVoltage(double volts) {
    claw_applied_volts = volts;
    clawSim.setInputVoltage(volts);
  }

    @Override
  public void setArmVoltage(double volts) {
    arm_applied_volts = volts;
    armSim.setInputVoltage(volts);
  }

  @Override
  public void goToAngle(double degrees, ClawIOInputs inputs, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      armGoal = new TrapezoidProfile.State(degrees, 0);
      armStartPoint = new TrapezoidProfile.State(inputs.arm_pos_deg, inputs.arm_velocityDegPS);
    }

    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(armSetpoint.position);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(armSim.getAngularPositionRad());
    setArmVoltage(calculatedVoltage);

    Logger.recordOutput("Claw/ArmSetpoint", armSetpoint.position);
  }

  @Override
  public void stop_arm() {
    setClawVoltage(0.0);
  }

  public void stop_claw() {
    setClawVoltage(0.0);
  }
}



