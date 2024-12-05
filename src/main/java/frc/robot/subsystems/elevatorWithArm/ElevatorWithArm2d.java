package frc.robot.subsystems.elevatorWithArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorWithArm2d {
  Mechanism2d mech;
  MechanismRoot2d root;
  public MechanismLigament2d elev;
  public MechanismLigament2d arm;
  ElevatorWithArm elevArm;
  ElevatorWithArmIOInputsAutoLogged input;
  public static ElevatorWithArm2d instance;

  public ElevatorWithArm2d() {
    mech = new Mechanism2d(4, 4);
    root = mech.getRoot("Root", 2, 0.5);

    elev = root.append(new MechanismLigament2d("elevator", 0.5, 90, 2, new Color8Bit(255, 0, 0)));
    arm = elev.append(new MechanismLigament2d("arm", 0.5, 90, 2, new Color8Bit(0, 255, 0)));
  }

  public void setHeight(double height) {
    elev.setLength(height);
  }

  public void setAngle(double angle) {
    arm.setAngle(Units.radiansToDegrees(angle));
  }

  public static ElevatorWithArm2d getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
        case SIM:
          instance = new ElevatorWithArm2d();
          break;
        case REPLAY:
          break;
        default:
          break;
      }
    }
    return instance;
  }

  public void periodic() {
    SmartDashboard.putData("Elevator With Arm 2D", mech);
    Logger.recordOutput("Elevator With Arm 2D", mech);
  }
}
