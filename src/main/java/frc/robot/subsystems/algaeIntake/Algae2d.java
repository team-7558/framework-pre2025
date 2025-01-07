package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Algae2d {
  private AlgaeIOInputsAutoLogged inputs;
  private Mechanism2d mech;
  private MechanismRoot2d root;
  private MechanismLigament2d frame;
  public MechanismLigament2d bb;
  public MechanismLigament2d motor;
  public static Algae2d instance;

  public Algae2d() {
    mech = new Mechanism2d(5, 5);
    root = mech.getRoot("Root", 1, 1);

    frame = root.append(new MechanismLigament2d("Frame", 2, 0, 1, new Color8Bit(138, 138, 138)));
    bb = root.append(new MechanismLigament2d("bb", 3, 90, 1, new Color8Bit(255, 0, 0)));
    motor = frame.append(new MechanismLigament2d("Motor", 1, 90, 1, new Color8Bit(255, 0, 0)));
  }

  public static Algae2d getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
        case SIM:
          instance = new Algae2d();
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
    SmartDashboard.putData("ALgae2d", mech);
    Logger.recordOutput("ALgae2d", mech);
  }
}
