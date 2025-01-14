package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Hang2d {
    private Mechanism2d mech;
    private MechanismLigament2d arm;
    private final String name;

    public Hang2d(String name, Color8Bit colour) {
        mech = new Mechanism2d(13, 13);
        this.name = name;

        MechanismRoot2d root = mech.getRoot("root", 7, 1);
        arm = root.append(new MechanismLigament2d("arm", 2, 90, 3, colour));
        MechanismLigament2d bottom = root.append(new MechanismLigament2d("bottom", 3, 0, 3, new Color8Bit(0, 100, 50)));
    }

    public void setAngle(double targetAngle) {
        arm.setAngle(targetAngle);
    }

    public void periodic() {
        SmartDashboard.putData(name, mech);
        Logger.recordOutput(name, mech);
    }
}
