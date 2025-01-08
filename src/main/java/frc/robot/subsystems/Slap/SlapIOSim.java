package frc.robot.subsystems.Slap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class SlapIOSim implements SlapIO {

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private final AltTimer timer = new AltTimer();

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
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(SlapIOInputs inputs) {
    // Update elevator simulation
    armSim.update(0.02); // 20 ms update
    inputs.pos_deg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.left_velDegPS = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    inputs.left_volts = applied_volts;
    inputs.left_currents =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setVoltage(double left_volts) {
    applied_volts = left_volts;
    armSim.setInputVoltage(left_volts);
  }

  @Override
  public void goToAngle(double degrees, SlapIOInputs inputs, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      armGoal = new TrapezoidProfile.State(degrees, 0);
      armStartPoint = new TrapezoidProfile.State(inputs.pos_deg, inputs.left_velDegPS);
    }

    armSetpoint = armProfile.calculate(timer.time(), armStartPoint, armGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(armSetpoint.position);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(armSim.getAngleRads());
    setVoltage(calculatedVoltage);

    Logger.recordOutput("Slap/ArmSetpoint", armSetpoint.position);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
