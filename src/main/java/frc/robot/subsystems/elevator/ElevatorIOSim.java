package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  ElevatorIOInputs temp = new ElevatorIOInputs();
  DCMotorSim elevSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);

  @Override
  public void updateInputs(ElevatorIOInputs Inputs) {
    elevSim.update(0.02);
  }

  public void setVoltage(double volts_V) {

  }

  public void setVel(double vel_mps) {
  }

  public void holdPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void travelToPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void resetPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void stop() {
    setVoltage(0.0);
  }

  public void toggleBrake() {
  }

  public void zero() {
  }

}
