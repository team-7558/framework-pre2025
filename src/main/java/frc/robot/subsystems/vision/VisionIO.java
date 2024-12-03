package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {


    @AutoLog
    public class VisionIOInputs {
        public Pose3d pose = new Pose3d();
        public double timestamp = 0;
        // all possible ones later
    }

    public default void updateInputs(VisionIOInputsAutoLogged inputs){}

    public void screenshot();

    public void ledControl(); // idk
}
