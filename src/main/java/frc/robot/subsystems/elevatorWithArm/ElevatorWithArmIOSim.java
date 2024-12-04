package frc.robot.subsystems.elevatorWithArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ElevatorWithArmIOSim implements ElevatorWithArmIO {
  private ElevatorSim sim =
      new ElevatorSim(DCMotor.getKrakenX60Foc(4), 25, 10, 0.05, 0.5, 3, false, 0.5);
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), 25, 10, 1, 0, 360, true, 180);
  private PIDController pidElev = new PIDController(100, 0, 0);
  private PIDController pidArm = new PIDController(100, 0, 0);
  private double appliedVoltsElev;
  private double appliedVoltsArm;
  private double elevPositionM;
  private double armPositionM;

  @Override
  public void updateInputs(ElevatorWithArmIOInputs inputs) {
    appliedVoltsElev = MathUtil.clamp(pidElev.calculate(sim.getPositionMeters()), -12.0, 12.0);
    sim.setInputVoltage(appliedVoltsElev);
    appliedVoltsArm = MathUtil.clamp(pidArm.calculate(armSim.getAngleRads()), -12.0, 12.0);
    armSim.setInputVoltage(appliedVoltsArm);
    elevPositionM = sim.getPositionMeters();
    armPositionM = armSim.getAngleRads();
  }

  public void setVoltageElev(double volts) {
    appliedVoltsElev = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVoltageArm(double volts) {
    appliedVoltsArm = volts;
    armSim.setInputVoltage(volts);
  }

  @Override
  public void TravelPosElev(double posM) {
    pidElev.setSetpoint(posM);
    setVoltageElev(pidElev.calculate(sim.getPositionMeters()));
  }

  @Override
  public void TravelAngleArm(double angleRad) {
    pidArm.setSetpoint(angleRad);
    setVoltageArm(pidArm.calculate(armSim.getAngleRads()));
  }

  @Override
  public void holdPosElev(double pos_m) {
    pidElev.setSetpoint(pos_m);
    setVoltageElev(pidElev.calculate(sim.getPositionMeters()));
  }

  @Override
  public void holdPosArm(double angleRad) {
    pidArm.setSetpoint(angleRad);
    setVoltageElev(pidArm.calculate(armSim.getAngleRads()));
  }

  @Override
  public void stop() {
    setVoltageElev(0.0);
    setVoltageArm(0.0);
  }
}
