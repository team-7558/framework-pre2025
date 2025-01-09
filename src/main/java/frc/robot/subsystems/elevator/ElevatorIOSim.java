package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

    ElevatorIOInputs temp = new ElevatorIOInputs();

    
  public void updateInputs(ElevatorIOInputs Inputs) {}

  public void setVoltage(double volts_V) {

  }

  public void setVel(double vel_mps) {}

  public void holdPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void travelToPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void resetPos(double pos_m) {
    temp.pos_m = pos_m;
  }

  public void stop() {}

  public void toggleBrake() {}

  public void zero() {} 




E


}
