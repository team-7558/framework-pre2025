package frc.robot.subsystems.pinkarm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Pinkarm2d {

  MechanismLigament2d arm;
  Mechanism2d mech;

  MechanismLigament2d bottom;
  private final String finalname;

  public Pinkarm2d(String name, Color8Bit color) {
    finalname = name;

    mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("root", 1.5, 0.5);

    arm = root.append(new MechanismLigament2d("arm", 0.5, 90, 10, color));
    bottom =
        root.append(new MechanismLigament2d("bottom", 0.001, 0, 20, new Color8Bit(125, 0, 125)));
  }

  public void setLength(double targetLength) {
    arm.setLength(targetLength);
  }

  public void setAngle(double targetAngle) {
    arm.setAngle(targetAngle);
  }

  public void periodic() {
    SmartDashboard.putData(finalname, mech);
    Logger.recordOutput(finalname, mech);
  }
}
