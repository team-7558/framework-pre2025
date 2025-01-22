package frc.robot.subsystems.hand;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Hand2d {
  Mechanism2d mech;
  MechanismRoot2d root;
  public MechanismLigament2d hand;
  MechanismLigament2d joint;
  MechanismLigament2d scoring;

  public static Hand2d instance;

  public Hand2d() {
    mech = new Mechanism2d(4, 4);
    root = mech.getRoot("Root", 2, 0.5);

    hand = root.append(new MechanismLigament2d("intake", 0.5, 90, 2, new Color8Bit(255, 0, 0)));
    joint = root.append(new MechanismLigament2d("joint", 1, 0, 2, new Color8Bit(255, 0, 255)));
    scoring =
        joint.append(new MechanismLigament2d("scoring", 0.5, 90, 2, new Color8Bit(255, 0, 0)));
  }

  public void setAngle(double angle) {
    hand.setAngle(angle);
  }

  public void score(double angle) {
    scoring.setAngle(angle);
  }

  public void periodic() {
    SmartDashboard.putData("Hand2d", mech);
    Logger.recordOutput("Hand2d", mech);
  }
}
