package frc.robot.subsystems.Slap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.AltTimer;

public class SlapIOSim implements SlapIO {

  private TrapezoidProfile.State armSetpoint;
  private TrapezoidProfile.State armStartPoint;
  private TrapezoidProfile.State armGoal;

  private final TrapezoidProfile.Constraints armConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

  private final AltTimer timer = new AltTimer();


  private final DCMotorSim motorSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), 45, 3.67); // Custom arm motor simulation
  private final PIDController armPositionPID = new PIDController(70, 1, 4);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(SlapIOInputs inputs) {
    // Update elevator simulation
    motorSim.update(0.02); // 20 ms update
    inputs.pos_deg = Units.radiansToDegrees(motorSim.getAngularPositionRad());
    inputs.left_velDegPS = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
    inputs.left_volts = applied_volts;
    inputs.left_currents =
        new double[] {
          motorSim.getCurrentDrawAmps(), motorSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setVoltage(double left_volts) {
    applied_volts = left_volts;
    motorSim.setInputVoltage(left_volts);
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
    double calculatedVoltage = armPositionPID.calculate(motorSim.getAngularPositionRad());
    setVoltage(calculatedVoltage);

    Logger.recordOutput("Slap/ArmSetpoint", armSetpoint.position);
  }




  @Override
  public void stop() {
    setVoltage(0.0);
  }

  
}
