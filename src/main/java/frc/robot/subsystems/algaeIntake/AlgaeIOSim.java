package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class AlgaeIOSim implements AlgaeIO {
  DCMotorSim motorSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Update elevator simulation
    motorSim.update(0.02); // 20 ms update
    inputs.VelocityDegPS = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());
    inputs.AppliedVolts = applied_volts;
    inputs.current_Amps =
        new double[] {
          motorSim.getCurrentDrawAmps(), motorSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setVoltage(double volts) {
    applied_volts = volts;
    motorSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
