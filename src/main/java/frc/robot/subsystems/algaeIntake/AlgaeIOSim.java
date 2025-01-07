package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeIOSim implements AlgaeIO {
  private DCMotorSim motor = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);
  private double algaeVolts;
  private double algaeAmps;
  public boolean beamBroken1;
  public boolean beamBroken2;
  private DigitalInput bb1 = new DigitalInput(1);
  private DigitalInput bb2 = new DigitalInput(2);

  public AlgaeIOSim() {
    beamBroken1 = false;
    beamBroken2 = false;
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    inputs.beamBroken1 = bb1.get();
    inputs.beamBroken2 = bb2.get();
    inputs.simOnlyRot = motor.getAngularPositionRotations();
    beamBroken1 = inputs.beamBroken1;
    beamBroken2 = inputs.beamBroken2;
  }

  /*
   * public default void setVoltage(double volts) {}
   * public default void breakBeamManual(boolean trueOrFalse) {}
   * public default void stop() {}
   */

  @Override
  public void setVoltage(double volts) {
    motor.setInputVoltage(volts);
  }

  @Override
  public void breakBeamManual(boolean trueOrFalse, int bb) {
    if (bb == 1) {
      beamBroken1 = trueOrFalse;
    } else if (bb == 2) {
      beamBroken2 = trueOrFalse;
    }
  }

  @Override
  public void stop() {
    motor.setInputVoltage(0);
  }
}
