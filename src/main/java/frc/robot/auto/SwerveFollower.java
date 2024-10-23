package frc.robot.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class SwerveFollower {

    private final PPHolonomicDriveController con;

    private final Drive drive;
    private final IFollowable toFollow;

    public SwerveFollower(IFollowable toFollow) {
        this.toFollow = toFollow;

        this.drive = Drive.getInstance();

        con = new PPHolonomicDriveController(
                Drive.TRANSLATION_CONSTANTS,
                Drive.ROTATION_CONSTANTS,
                Constants.globalDelta_sec);
    }

    public void start() {
        if (!toFollow.isGenerated()) {
            toFollow.generate();
        }

        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();

        con.reset(currentPose, currentSpeeds);

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
        Rotation2d currentHeading = new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Rotation2d targetHeading = toFollow.getInitState().pose.getRotation();
        Rotation2d headingError = currentHeading.minus(targetHeading);
        boolean onHeading = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
                || Math.abs(headingError.getDegrees()) < 30;

    }

    public void step(double t) {
        PathPlannerTrajectoryState targetState = toFollow.sample(t);


        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();


        ChassisSpeeds targetSpeeds = con.calculateRobotRelativeSpeeds(currentPose, targetState);

        /*
         * if (G.isRedAlliance()){
         * targetSpeeds.vxMetersPerSecond *= -1.0;
         * targetSpeeds.vyMetersPerSecond *= -1.0;
         * }
         */

        double currentVel = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        PPLibTelemetry.setCurrentPose(currentPose);
        PathPlannerLogging.logCurrentPose(currentPose);

        PPLibTelemetry.setTargetPose(targetState.pose);
        PathPlannerLogging.logTargetPose(targetState.pose);

        PPLibTelemetry.setVelocities(
                currentVel,
                targetState.linearVelocity,
                currentSpeeds.omegaRadiansPerSecond,
                targetSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("SwerveFollower/TargetSpeeds", targetSpeeds);

        drive.runVelocity(targetSpeeds);
    }

    public boolean elapsed(double t) {
        return t >= toFollow.endTime();
    }

    public void end(boolean interrupted) {
        // Only output 0 speeds when ending a path that is supposed to stop, this allows
        // interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && toFollow.getEndState().linearVelocity < 0.05) {
            drive.runVelocity(new ChassisSpeeds());
        }

        PathPlannerLogging.logActivePath(null);
    }
}