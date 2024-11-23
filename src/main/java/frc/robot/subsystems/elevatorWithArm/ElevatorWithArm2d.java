package frc.robot.subsystems.elevatorWithArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorWithArm2d {
    Mechanism2d mech;
    MechanismRoot2d root;
    public ElevatorWithArm2d() {
        mech = new Mechanism2d(4,4);
        root = mech.getRoot("Root", 2, 0.5);
        
        MechanismLigament2d elev = root.append(new MechanismLigament2d("elevator",0.5,90,2,new Color8Bit(255,0,0)));
        MechanismLigament2d arm = elev.append(new MechanismLigament2d("arm",0.5,90,2,new Color8Bit(0, 255, 0)));
    }
    public void periodic() {
        SmartDashboard.putData("Elevator With Arm 2D", mech);
        Logger.recordOutput("Elevator With Arm 2D", mech);
    }
}
