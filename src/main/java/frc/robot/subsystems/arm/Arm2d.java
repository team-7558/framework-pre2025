package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Arm2d {

  private final MechanismLigament2d Sideview;
  private final MechanismLigament2d FrontView;
  private final Mechanism2d mech;
  private final String finalName;

  /**
   * Constructs a new Pinkarm2d visualization with a given name and color.
   *
   * @param name The name of the mechanism (used for display in SmartDashboard and logs).
   * @param color The color of the arm ligament in the visualization.
   */
  public Arm2d(String name, Color8Bit color) {
    this.finalName = name;

    mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("root", 0.5, 0.5);

    MechanismLigament2d connection =
        root.append(new MechanismLigament2d("connector", 1.5, 0, 0.05, new Color8Bit(0, 0, 0)));

    Sideview = root.append(new MechanismLigament2d("SideView", 0.5, 90, 10, color));

    FrontView = connection.append(new MechanismLigament2d("FrontView", 0.5, 90, 10, color));

    MechanismLigament2d bottom =
        root.append(new MechanismLigament2d("bottom", 0.001, 0, 20, new Color8Bit(125, 0, 125)));

    MechanismLigament2d bottom2 =
        connection.append(
            new MechanismLigament2d("bottom2", 0.001, 0, 20, new Color8Bit(125, 0, 125)));
  }

  /**
   * Sets the length of the arm in the visualization.
   *
   * <p>/** Sets the angle of the arm in the visualization.
   *
   * @param targetAngle The desired angle of the arm.
   */
  public void setElbowAngle(double targetAngle) {
    Sideview.setAngle(targetAngle);
    FrontView.setLength(0.5 * Math.cos(Math.toRadians(targetAngle)));
  }

  public void setShoulderAngle(double targetAngle) {
    FrontView.setAngle(targetAngle);
    Sideview.setLength(0.5 * Math.cos(Math.toRadians(targetAngle)));
  }

  /** Periodically updates the SmartDashboard and logs the mechanism state. */
  public void periodic() {
    SmartDashboard.putData(finalName, mech);
    Logger.recordOutput(finalName, mech);
  }
}
