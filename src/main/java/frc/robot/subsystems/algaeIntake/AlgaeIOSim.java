package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AlgaeIOSim implements AlgaeIO {
  private SingleJointedArmSim motor = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1),1,1,0,0,360,false,0);
  private double algaeVolts;
  private double algaeAmps;
  public boolean beamBroken1;
  public boolean beamBroken2;
  private Algae2d mech;
  private DigitalInput bb1 = new DigitalInput(1);

  public AlgaeIOSim() {
    beamBroken1 = false;
    beamBroken2 = false;
    mech = Algae2d.getInstance();
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    mech.motor.setAngle(inputs.simOnlyRot);
    if (inputs.beamBroken1) {
      mech.bb.setColor(new Color8Bit(0, 255, 0));
    }
    inputs.beamBroken1 = bb1.get();
    inputs.simOnlyRot = motor.getAngleRads();
    beamBroken1 = inputs.beamBroken1;
    inputs.algaeVolts = algaeVolts;
    inputs.algaeSpeed = motor.getVelocityRadPerSec();
  }

  /*
   * public default void setVoltage(double volts) {}
   * public default void breakBeamManual(boolean trueOrFalse) {}
   * public default void stop() {}
   */

  @Override
  public void setVoltage(double volts) {
    algaeVolts = volts;
    motor.setInputVoltage(volts);
  }

  @Override
  public void breakBeamManual(boolean trueOrFalse) {
    beamBroken1 = trueOrFalse;
  }

  @Override
  public void stop() {
    motor.setInputVoltage(0);
  }
}
