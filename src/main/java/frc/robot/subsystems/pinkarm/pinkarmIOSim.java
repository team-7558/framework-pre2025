package frc.robot.subsystems.pinkarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.pinkarm.PinkarmIO.PinkarmInputs;

public class PinkarmIOSim implements PinkarmIO {

  private ElevatorSim ElevSim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(1),
          45,
          20,
          0.5,
          Pinkarm.ELEV_MIN_HEIGHT_M,
          Pinkarm.ELEV_MAX_HEIGHT_M,
          false,
          5,
          null);

  private DCMotorSim ArmSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 45, 3.67);
  private PIDController elev_posPid = new PIDController(0.0, 0.0, 0.0);
  private double elev_appliedVolts = 0.0;
  private double arm_appliedVolts = 0.0;

  private final PIDController arm_posPid = new PIDController(5.0, 0.0, 0.0);

  @Override
  public void updateInputs(PinkarmInputs inputs) {
    // Update inputs based on current state
    ElevSim.update(0.05);
    inputs.elev_posMeters = ElevSim.getPositionMeters();
    inputs.elev_velMPS = ElevSim.getVelocityMetersPerSecond();
    inputs.elev_volts = elev_appliedVolts;
    inputs.elev_currents =
        new double[] {ElevSim.getCurrentDrawAmps(), ElevSim.getCurrentDrawAmps()};

    ArmSim.update(0.05);
    inputs.arm_posDegrees = Math.toDegrees(ArmSim.getAngularPositionRad());
    inputs.arm_velDegPS = Math.toDegrees(ArmSim.getAngularVelocityRadPerSec());
    inputs.arm_volts = arm_appliedVolts;
    inputs.arm_currents = new double[] {ArmSim.getCurrentDrawAmps(), ArmSim.getCurrentDrawAmps()};
  }

  @Override
  public void setelevVoltage(double volts) {
    elev_appliedVolts = volts;
    ElevSim.setInputVoltage(volts);
  }

  @Override
  public void setArmVoltage(double volts) {
    arm_appliedVolts = volts;
    ArmSim.setInputVoltage(volts);
  }

  @Override
  public void goToAngle(double degrees) {
    arm_posPid.setSetpoint(degrees);
    setArmVoltage(arm_posPid.calculate(ArmSim.getAngularPositionRad()));
  }

  @Override
  public void goToPos(double meters) {
    elev_posPid.setSetpoint(meters);
    setelevVoltage(elev_posPid.calculate(ElevSim.getPositionMeters()));
  }

  @Override
  public void stop() {
    setelevVoltage(0.0);
  }
}
