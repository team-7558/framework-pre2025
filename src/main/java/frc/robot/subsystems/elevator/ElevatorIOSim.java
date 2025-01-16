package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorIOInputs temp = new ElevatorIOInputs();
  private ElevatorSim elev =
      new ElevatorSim(DCMotor.getKrakenX60Foc(1), 25, 10, 0.05, 0.5, 3, false, 0.5);

  private PIDController pidE = new PIDController(100, 0, 0);

  private double appliedVoltsElev;
  

  public void updateInputs(ElevatorIOInputs Inputs) {}

  public void setVoltage(double volts_V) {}

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
