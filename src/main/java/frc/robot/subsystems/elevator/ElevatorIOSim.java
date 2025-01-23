package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorIOInputs temp = new ElevatorIOInputs();
  private ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60Foc(1), 25, 10, 0.05, 0.5, 3, false, 0.5);

  private PIDController pidE = new PIDController(100, 0, 0);


  public void updateInputs(ElevatorIOInputs inputs) {
    elevSim.update(Constants.globalDelta_s);
    inputs.volts_V = MathUtil.clamp(pidE.calculate(elevSim.getPositionMeters()), -12.0, 12.0);
    inputs.vel_mps = elevSim.getVelocityMetersPerSecond();
    inputs.pos_m = elevSim.getPositionMeters();


  }

  public void setVoltage(double volts_V) {
    elevSim.setInputVoltage(volts_V);
  }

  public void setVel(double vel_mps) {
    pidE.setSetpoint(vel_mps);
    setVoltage(pidE.calculate(elevSim.getVelocityMetersPerSecond()));
    
  }

  public void holdPos(double pos_m) {
    temp.pos_m = pos_m;
    pidE.setSetpoint(pos_m);

  }

  public void travelToPos(double pos_m) {
    temp.pos_m = pos_m;
    setVoltage(pidE.calculate(elevSim.getPositionMeters()));
  }

  public void resetPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void stop() {
    setVoltage(0.0);
  }

  public void toggleBrake() {}


}
