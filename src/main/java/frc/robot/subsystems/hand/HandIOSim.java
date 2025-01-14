package frc.robot.subsystems.hand;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HandIOSim implements HandIO {
  public FlywheelSim hand = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(HandIOInputs inputs) {
    // Update elevator simulation
    hand.update(0.02); // 20 ms update
    inputs.VelocityDegPS = Units.radiansToDegrees(hand.getAngularVelocityRadPerSec());
    inputs.AppliedVolts = applied_volts;
    inputs.current_Amps =
        new double[] {
          hand.getCurrentDrawAmps(), hand.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currentsFF
  }

  @Override
  public void setVoltage(double volts) {
    applied_volts = volts;
    hand.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
