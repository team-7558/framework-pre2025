package frc.robot.subsystems.Slap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SlapIOSim implements SlapIO {
  private final DCMotorSim motorSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), 45, 3.67); // Custom arm motor simulation
  private final PIDController armPositionPID = new PIDController(70, 1, 4);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(SlapIOInputs inputs) {
    // Update elevator simulation
    motorSim.update(0.02); // 20 ms update
    inputs.pos_deg = Units.radiansToDegrees(motorSim.getAngularPositionRad());
    inputs.velDegPS = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
    inputs.volts = applied_volts;
    inputs.currents =
        new double[] {
          motorSim.getCurrentDrawAmps(), motorSim.getCurrentDrawAmps()
        }; // Simulate multiple motor currents
  }

  @Override
  public void setVoltage(double volts) {
    applied_volts = volts;
    motorSim.setInputVoltage(volts);
  }

  @Override
  public void goToAngle(double degrees) {
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(degrees);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(motorSim.getAngularPositionRad());
    setVoltage(calculatedVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
