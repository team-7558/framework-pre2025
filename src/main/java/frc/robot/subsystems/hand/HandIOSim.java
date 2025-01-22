package frc.robot.subsystems.hand;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class HandIOSim implements HandIO {
  public FlywheelSim intake = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  public FlywheelSim scoring = new FlywheelSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private double intake_applied_volts = 0.0;
  private double scoring_applied_volts = 0.0;

  @Override
  public void updateInputs(HandIOInputs inputs) {
    // Update elevator simulation
    intake.update(0.02); // 20 ms update
    scoring.update(0.02); // 20 ms update
    inputs.intakeVelocityDegPS = Units.radiansToDegrees(intake.getAngularVelocityRadPerSec());
    inputs.intakeAppliedVolts = intake_applied_volts;
    inputs.intake_current_Amps =
        new double[] {
          intake.getCurrentDrawAmps(), intake.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currentsFF
    inputs.scoringVelocityDegPS = Units.radiansToDegrees(scoring.getAngularVelocityRadPerSec());
    inputs.scoringAppliedVolts = scoring_applied_volts;
    inputs.scoring_current_Amps =
        new double[] {
          scoring.getCurrentDrawAmps(), scoring.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currentsFF
  }

  @Override
  public void intakeSetVoltage(double volts) {
    intake_applied_volts = volts;
    intake.setInputVoltage(volts);
  }

  @Override
  public void scoringSetVoltage(double volts) {
    scoring_applied_volts = volts;
    scoring.setInputVoltage(volts);
  }

  @Override
  public void intakeStop() {
    intakeSetVoltage(0.0);
  }

  @Override
  public void scoringStop() {
    intakeSetVoltage(0.0);
  }
}
