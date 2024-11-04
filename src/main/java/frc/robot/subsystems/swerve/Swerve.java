package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.swerve.Module.Mode;
import frc.robot.util.ChassisAcceleration;
import frc.robot.util.Util;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends StateMachineSubsystemBase<PathingMode> {

  public static final SwerveConfigVentura CFG = new SwerveConfigVentura();
  public static final Lock odometryLock = new ReentrantLock();
  public static Swerve instance;

  public static Swerve getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
        case SIM:
        default:
          instance =
              new Swerve(
                  new GyroIO() {}, new ModuleIO() {},
                  new ModuleIO() {}, new ModuleIO() {},
                  new ModuleIO() {}, null);
          break;
      }
      System.out.println("Swerve Initialized");
    }
    return instance;
  }

  private final SwerveInput input;
  private ChassisSpeeds inputSpeeds;
  private ChassisSpeeds outputSpeeds;
  private ChassisSpeeds acc;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SwerveDriveSimulation sim;

  private SwerveDriveKinematics kin;
  private Rotation2d rawYaw_Rot2d;
  private SwerveModulePosition[] prevModulePos; // For delta tracking
  private Translation2d centerOfRotation;
  private SwerveDrivePoseEstimator poseEstimator;

  private PathingOverride override;

  private Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      SwerveDriveSimulation driveSim) {
    super("Swerve");
    this.sim = driveSim;
    this.gyroIO = gyroIO;
    this.modules[CFG.FL] = new Module(flModuleIO, CFG.FL);
    this.modules[CFG.FR] = new Module(frModuleIO, CFG.FR);
    this.modules[CFG.BL] = new Module(blModuleIO, CFG.BL);
    this.modules[CFG.BR] = new Module(brModuleIO, CFG.BR);

    kin = new SwerveDriveKinematics(getModuleTranslations());
    rawYaw_Rot2d = new Rotation2d();
    prevModulePos =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    centerOfRotation = new Translation2d(CFG.COR_OFFSET_X_m, CFG.COR_OFFSET_Y_m);
    poseEstimator = new SwerveDrivePoseEstimator(kin, rawYaw_Rot2d, prevModulePos, new Pose2d());

    // TODO: Start threads

    // TODO: Autobuilder config

    // Simulated Arena
    // SimulatedArena.getInstance()
    //    .addDriveTrainSimulation(sim); // register the drive train simulation

    // reset the field for auto (placing game-pieces in positions)
    // SimulatedArena.getInstance().resetFieldForAuto();

    input = new SwerveInput(SwerveInput.ZERO);
    inputSpeeds = new ChassisSpeeds();
    outputSpeeds = new ChassisSpeeds();
    acc = new ChassisSpeeds();
    override = PathingOverride.NONE;
    queueState(PathingMode.DISABLED);
  }

  @Override
  protected void inputPeriodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data

    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }

    odometryLock.unlock();

    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (var module : modules) {
      module.inputPeriodic();
    }

    // Update Odometry before handling states
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePos = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePos[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePos[moduleIndex].distanceMeters - prevModulePos[moduleIndex].distanceMeters,
                modulePos[moduleIndex].angle);
        prevModulePos[moduleIndex] = modulePos[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawYaw_Rot2d = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kin.toTwist2d(moduleDeltas);
        rawYaw_Rot2d = rawYaw_Rot2d.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawYaw_Rot2d, modulePos);
    }
  }

  @Override
  public void handleStateMachine() {
    double vxi, vyi, wi; // Input translational and rotational velocities

    // Calculate throttle speed
    double maxLinearVel_mps =
        Util.lerp(CFG.MAX_LINEAR_VEL_THROTTLED_mps, CFG.MAX_LINEAR_VEL_mps, input.throttle);
    double maxAngularVel_radps =
        Util.lerp(CFG.MAX_ANGULAR_VEL_THROTTLED_radps, CFG.MAX_ANGULAR_VEL_radps, input.throttle);

    // Circular input processing
    double inputMagnitude = Math.hypot(input.xi, input.yi);
    vxi = input.xi * maxLinearVel_mps;
    vyi = input.yi * maxLinearVel_mps;
    wi = input.wi * maxAngularVel_radps;

    if (inputMagnitude > 1.0) {
      vxi = vxi / inputMagnitude;
      vyi = vyi / inputMagnitude;
    }

    inputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxi, vyi, wi, getRotation());
    switch (getState()) {
      case DISABLED:
        break;
      case FIELD_RELATIVE:
        break;
      case POSE_FOLLOWING:
        break;
      case PATH_FOLLOWING:
        break;
      default:
        break;
    }

    ChassisSpeeds measuredSpeeds = getChassisSpeeds();
    ChassisSpeeds inputAcc =
        ChassisAcceleration.fromChassisSpeeds(
            measuredSpeeds, inputSpeeds, Constants.globalDelta_ms);

    acc = accLimit(inputAcc);
    outputSpeeds =
        ChassisAcceleration.fromAcceleration(measuredSpeeds, acc, Constants.globalDelta_ms);

    // Log speeds and accelerations
    Logger.recordOutput("Swerve/Speeds/InputVel", inputSpeeds);
    Logger.recordOutput("Swerve/Speeds/MeasuredVel", measuredSpeeds);
    Logger.recordOutput("Swerve/Speeds/OutputVel", outputSpeeds);
    Logger.recordOutput("Swerve/Speeds/InputAcc", inputAcc);
    Logger.recordOutput("Swerve/Speeds/OutputAcc", acc);
  }

  @Override
  protected void outputPeriodic() {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(outputSpeeds, Constants.globalDelta_ms);
    SwerveModuleState[] setpointStates = kin.toSwerveModuleStates(discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates,
        discreteSpeeds,
        CFG.MAX_LINEAR_VEL_mps,
        CFG.MAX_LINEAR_VEL_mps,
        CFG.MAX_ANGULAR_VEL_radps);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    double maxModuleVel = 0.0;
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].writeState(setpointStates[i]);
      if (Math.abs(optimizedSetpointStates[i].speedMetersPerSecond)
          > CFG.MAX_LINEAR_VEL_CONTROLLED_mps) {
        maxModuleVel = Math.abs(optimizedSetpointStates[i].speedMetersPerSecond);
      }
    }

    // Determine if openloop control should be used
    Module.Mode mode =
        (maxModuleVel > CFG.MAX_LINEAR_VEL_CONTROLLED_mps) ? Mode.HIGH_SPEED : Mode.HIGH_CONTROL;

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    } else {
      for (var module : modules) {
        module.outputPeriodic(mode);
      }
    }

    // Log setpoint states
    Logger.recordOutput("Swerve/Modules/Mode", mode);
    Logger.recordOutput("Swerve/Modules/Setpoints", setpointStates);
    Logger.recordOutput("Swerve/Modules/SetpointsOptimized", optimizedSetpointStates);
  }

  public void setInput(SwerveInput i) {
    input.set(i);
  }

  public void setOverride(PathingOverride override) {
    this.override = override;
  }

  public ChassisSpeeds accLimit(ChassisSpeeds in) {
    return in;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Swerve/Odom")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public double getAngularVelocity() {
    if (gyroInputs.connected) {
      return gyroInputs.yawVel_radps;
    } else {
      return getChassisSpeeds().omegaRadiansPerSecond;
    }
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawYaw_Rot2d, getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kin.toChassisSpeeds(getModuleStates());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Swerve/Modules/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] pos = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      pos[i] = modules[i].getPosition();
    }
    return pos;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    Translation2d[] translations = new Translation2d[4];
    translations[CFG.FL] = new Translation2d(CFG.TRACK_WIDTH_X_m / 2.0, CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[CFG.FR] = new Translation2d(CFG.TRACK_WIDTH_X_m / 2.0, -CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[CFG.BL] = new Translation2d(-CFG.TRACK_WIDTH_X_m / 2.0, CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[CFG.BR] =
        new Translation2d(-CFG.TRACK_WIDTH_X_m / 2.0, -CFG.TRACK_WIDTH_Y_m / 2.0);
    return translations;
  }
}
