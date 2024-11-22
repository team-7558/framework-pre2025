package frc.robot.subsystems.pinkarm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class pinkarm2d {

  MechanismLigament2d arm;
  Mechanism2d mech;

  public pinkarm2d() {
    mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("root", 1.5, 0.5);

    arm = root.append(new MechanismLigament2d("arm", 0.5, 90));
  }

  public void setLength(double targetLength) {
    arm.setLength(targetLength);
  }

  public void setAngle(double targetAngle) {
    arm.setAngle(targetAngle);
  }

  public void periodic() {
    SmartDashboard.putData("pinkarm2d", mech);
    Logger.recordOutput("pinkarm2d", mech);
  }
}
