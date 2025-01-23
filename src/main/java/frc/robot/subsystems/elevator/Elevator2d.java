package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Elevator2d {

  Mechanism2d mech;
  MechanismRoot2d root;

  public MechanismLigament2d elev;

  Elevator elevator;

  public static Elevator2d instance;

  public Elevator2d() {
    mech = new Mechanism2d(4, 4);
    root = mech.getRoot("Root", 2, 0.5);
    elev = root.append(new MechanismLigament2d("elevator", 0.5, 90, 2, new Color8Bit(0, 255, 255)));
  }

  public void setHeight(double height) {
    elev.setLength(height);
  }

  public void periodic() {
    SmartDashboard.putData("Elevator2D", mech);
    Logger.recordOutput("Elevator2D", mech);
  }
}
