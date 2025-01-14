// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.drive.Module.Mode;
import frc.robot.util.ChassisAcceleration;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Util;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase<PathingMode> {
  // Indexing
  public static final int FL = 0, FR = 1, BL = 2, BR = 3;

  public static final SwerveConfigBigRed CFG = new SwerveConfigBigRed();
  public static final Lock odometryLock = new ReentrantLock();
  private static Drive instance;

  public static Drive getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(
                      CFG.FL_ID_DRIVE,
                      CFG.FL_ID_STEER,
                      CFG.FL_ID_CANCODER,
                      CFG.FL_ABS_OFFSET,
                      CFG.FL_INVERT_DRIVE,
                      CFG.FL_INVERT_STEER),
                  new ModuleIOTalonFX(
                      CFG.FR_ID_DRIVE,
                      CFG.FR_ID_STEER,
                      CFG.FR_ID_CANCODER,
                      CFG.FR_ABS_OFFSET,
                      CFG.FR_INVERT_DRIVE,
                      CFG.FR_INVERT_STEER),
                  new ModuleIOTalonFX(
                      CFG.BL_ID_DRIVE,
                      CFG.BL_ID_STEER,
                      CFG.BL_ID_CANCODER,
                      CFG.BL_ABS_OFFSET,
                      CFG.BL_INVERT_DRIVE,
                      CFG.BL_INVERT_STEER),
                  new ModuleIOTalonFX(
                      CFG.BR_ID_DRIVE,
                      CFG.BR_ID_STEER,
                      CFG.BR_ID_CANCODER,
                      CFG.BR_ABS_OFFSET,
                      CFG.BR_INVERT_DRIVE,
                      CFG.BR_INVERT_STEER),
                  null);
          break;
        case SIM:
          final GyroSimulation gyroSimulation = GyroSimulation.createPigeon2();

          SwerveDriveSimulation swerveDriveSimulation =
              new SwerveDriveSimulation(
                  CFG.MASS_KG,
                  CFG.TRACK_WIDTH_Y_m,
                  CFG.TRACK_WIDTH_X_m,
                  CFG.BOT_WIDTH_Y_m,
                  CFG.BOT_WIDTH_X_m,
                  SwerveModuleSimulation.getMark4i( // creates a mark4 module
                      DCMotor.getKrakenX60Foc(1), // drive motor is a Kraken x60
                      DCMotor.getKrakenX60Foc(1), // steer motor is a Falcon 500
                      CFG.DRIVE_SUPPLY_LIMIT_A, // current limit: 80 Amps
                      DRIVE_WHEEL_TYPE.TIRE, // rubber wheels
                      2 // l2 gear ratio
                      ),
                  gyroSimulation,
                  Poses.START);

          instance =
              new Drive(
                  new GyroIOSim(
                      gyroSimulation), // GyroIOSim is a wrapper around gyro simulation, that reads
                  // the simulation result
                  /* ModuleIOSim are edited such that they also wraps around module simulations */
                  new ModuleIOSim(swerveDriveSimulation.getModules()[FL]),
                  new ModuleIOSim(swerveDriveSimulation.getModules()[FR]),
                  new ModuleIOSim(swerveDriveSimulation.getModules()[BL]),
                  new ModuleIOSim(swerveDriveSimulation.getModules()[BR]),
                  swerveDriveSimulation);
          break;
        default:
          instance =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  null);
          break;
      }
    }
    return instance;
  }

  private final SwerveInput si;
  private ChassisSpeeds inputSpeeds;
  private ChassisSpeeds outputSpeeds;
  private ChassisSpeeds acc;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private SwerveDriveSimulation sim;

  private double autolockSetpoint_r = 0, intermediaryAutolockSetpoint_r = 0;

  private SwerveDriveKinematics kin;
  private Rotation2d rawYaw_Rot2d;
  private SwerveModulePosition[] prevModulePos; // For delta tracking
  private Translation2d centerOfRotation;
  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveOdometry odom;

  private PathingOverride override;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      SwerveDriveSimulation driveSim) {

    super("Drive");
    sim = driveSim;
    this.gyroIO = gyroIO;
    this.modules[FL] = new Module(flModuleIO, FL);
    this.modules[FR] = new Module(frModuleIO, FR);
    this.modules[BL] = new Module(blModuleIO, BL);
    this.modules[BR] = new Module(brModuleIO, BR);

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

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
    poseEstimator = new SwerveDrivePoseEstimator(kin, rawYaw_Rot2d, prevModulePos, Poses.START);
    odom = new SwerveDriveOdometry(kin, rawYaw_Rot2d, prevModulePos, Poses.START);
    AutoBuilder.configure(
        () -> getPose(),
        this::setPose,
        () -> getChassisSpeeds(),
        (speeds, feedforwards) -> run(speeds),
        new PPHolonomicDriveController(new PIDConstants(5.0, 0, 0), new PIDConstants(5.0, 0, 0)),
        config,
        () -> shouldFlip(),
        null);

    // Start threads (no-op for each if no signals have been created)

    // Configure AutoBuilder for PathPlanner

    // Simulation

    if (sim != null) {
      SimulatedArena.getInstance()
          .addDriveTrainSimulation(sim); // register the drive train simulation

      // reset the field for auto (placing game-pieces in positions)
      // SimulatedArena.getInstance().resetFieldForAuto();
      SimulatedArena.getInstance().clearGamePieces();
    }
    System.out.println("STARTING THREAD");
    // PhoenixOdometryThread.getInstance().start();

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    si = new SwerveInput(SwerveInput.ZERO);
    inputSpeeds = new ChassisSpeeds();
    outputSpeeds = new ChassisSpeeds();
    acc = new ChassisSpeeds();
    override = PathingOverride.NONE;
    queueState(PathingMode.DISABLED);
  }

  public void run(ChassisSpeeds run) {
    setInputSpeeds(run);
  }

  public boolean shouldFlip() {
    return true;
  }

  @Override
  public void inputPeriodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.inputPeriodic();
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - prevModulePos[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        prevModulePos[moduleIndex] = modulePositions[moduleIndex];
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
      poseEstimator.updateWithTime(sampleTimestamps[i], rawYaw_Rot2d, modulePositions);
      odom.update(rawYaw_Rot2d, modulePositions);
    }
  }

  @Override
  public void handleStateMachine() {

    // Calculate throttle speed

    switch (getState()) {
      case DISABLED:
        if (stateInit()) { // First time init stuff per entry of state
        }
        break;
      case FIELD_RELATIVE:
        System.out.println("field relative");
        double maxLinearVel_mps =
            Util.lerp(CFG.MAX_LINEAR_VEL_THROTTLED_mps, CFG.MAX_LINEAR_VEL_mps, si.throttle);
        double maxAngularVel_radps =
            Util.lerp(CFG.MAX_ANGULAR_VEL_THROTTLED_radps, CFG.MAX_ANGULAR_VEL_radps, si.throttle);

        // Circular input processing
        double inputMagnitude = Math.hypot(si.xi, si.yi);

        double x_ = si.xi * maxLinearVel_mps;
        double y_ = si.yi * maxLinearVel_mps;
        double w_ = si.wi * maxAngularVel_radps;

        if (inputMagnitude > 1.0) {
          x_ = x_ / inputMagnitude;
          y_ = y_ / inputMagnitude;
        }

        inputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_, y_, w_, getRotation());
        break;
      case POSE_FOLLOWING:
        break;
      case PATH_FOLLOWING:
        // System.out.println("path following");
        break;
      default:
    }

    ChassisSpeeds measuredSpeeds = getChassisSpeeds();
    ChassisSpeeds inputAcc =
        ChassisAcceleration.fromChassisSpeeds(measuredSpeeds, inputSpeeds, Constants.globalDelta_s);

    acc = inputAcc;
    // acc = accLimitForward(acc, measuredSpeeds);
    // acc = accLimitAngular(acc);
    // acc = accLimitTilt(acc);
    // acc = accLimitSkid(acc);

    outputSpeeds =
        ChassisAcceleration.fromAcceleration(measuredSpeeds, acc, Constants.globalDelta_s);

    // Log speeds and accelerations
    Logger.recordOutput("Drive/Speeds/Input", new ChassisSpeeds(si.xi, si.yi, si.wi));
    Logger.recordOutput("Drive/Speeds/InputVel", inputSpeeds);
    Logger.recordOutput("Drive/Speeds/MeasuredVel", measuredSpeeds);
    Logger.recordOutput("Drive/Speeds/OutputVel", outputSpeeds);
    Logger.recordOutput("Drive/Speeds/InputAcc", inputAcc);
    Logger.recordOutput("Drive/Speeds/OutputAcc", acc);
  }

  @Override
  public void outputPeriodic() {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(outputSpeeds, Constants.globalDelta_s);
    SwerveModuleState[] setpointStates = kin.toSwerveModuleStates(discreteSpeeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CFG.MAX_LINEAR_VEL_mps);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    double maxModuleVel = 0.0;
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
      if (Math.abs(optimizedSetpointStates[i].speedMetersPerSecond)
          > CFG.MAX_LINEAR_VEL_CONTROLLED_mps) {
        maxModuleVel = Math.abs(optimizedSetpointStates[i].speedMetersPerSecond);
      }
    }

    // Determine if openloop control should be used
    Module.Mode mode =
        (maxModuleVel > CFG.MAX_LINEAR_VEL_CONTROLLED_mps) ? Mode.HIGH_SPEED : Mode.HIGH_CONTROL;

    // Stop moving when disabled
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
    Logger.recordOutput("Drive/Modules/Mode", mode);
    Logger.recordOutput("Drive/Modules/Setpoints", setpointStates);
    Logger.recordOutput("Drive/Modules/SetpointsOptimized", optimizedSetpointStates);
  }

  public void setInput(SwerveInput i) {
    si.set(i);
  }

  public void setInputSpeeds(ChassisSpeeds speeds) {
    this.inputSpeeds = speeds;
  }

  public ChassisSpeeds accLimitForward(ChassisSpeeds acc, ChassisSpeeds vel) {
    ChassisSpeeds res = acc;
    double accMag_mps2 = ChassisAcceleration.magnitude(acc);

    double velMag_mps = ChassisAcceleration.magnitude(vel);

    // TODO: cosine profile with direction of vel, or dot product

    double maxFwdAcc = CFG.MAX_FORWARD_ACC_mps2 * (1.0 - velMag_mps / CFG.MAX_LINEAR_VEL_mps);
    double outMag = Math.min(accMag_mps2, maxFwdAcc);
    if (accMag_mps2 != 0.0) {
      res = acc.times(outMag / accMag_mps2);
    }
    return res;
  }

  public ChassisSpeeds accLimitAngular(ChassisSpeeds in) {
    ChassisSpeeds res = in;

    double angularAcc_radps2 =
        Math.copySign(
            Math.min(CFG.MAX_ANGULAR_ACC_radps2, Math.abs(in.omegaRadiansPerSecond)),
            in.omegaRadiansPerSecond);
    res.omegaRadiansPerSecond = angularAcc_radps2;

    return res;
  }

  public ChassisSpeeds accLimitTilt(ChassisSpeeds in) {
    ChassisSpeeds res = in;

    res.vxMetersPerSecond =
        Util.limit(in.vxMetersPerSecond, -CFG.MAX_TILT_XNEG_ACC_mps2, CFG.MAX_TILT_XPOS_ACC_mps2);
    res.vyMetersPerSecond =
        Util.limit(in.vyMetersPerSecond, -CFG.MAX_TILT_YNEG_ACC_mps2, CFG.MAX_TILT_YPOS_ACC_mps2);

    return res;
  }

  public ChassisSpeeds accLimitSkid(ChassisSpeeds acc) {
    ChassisSpeeds res = acc;
    double accMag_mps2 = ChassisAcceleration.magnitude(acc);

    double outMag = Math.min(accMag_mps2, CFG.MAX_SKID_ACC_mps2);
    if (accMag_mps2 != 0.0) {
      res = acc.times(outMag / accMag_mps2);
    }

    return res;
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kin.resetHeadings(headings);
    // stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/Modules/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Drive/Odom")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
    // return odom.getPoseMeters();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return gyroInputs.yaw_Rot2d;
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
    odom.resetPosition(rawYaw_Rot2d, getModulePositions(), pose);
  }

  public void zeroGyro() {
    gyroIO.zero();
    Pose2d p = getPose();
    setPose(new Pose2d(p.getTranslation(), Rotation2d.fromRotations(0)));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kin.toChassisSpeeds(getModuleStates());
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    Translation2d[] translations = new Translation2d[4];
    translations[FL] = new Translation2d(CFG.TRACK_WIDTH_X_m / 2.0, CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[FR] = new Translation2d(CFG.TRACK_WIDTH_X_m / 2.0, -CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[BL] = new Translation2d(-CFG.TRACK_WIDTH_X_m / 2.0, CFG.TRACK_WIDTH_Y_m / 2.0);
    translations[BR] = new Translation2d(-CFG.TRACK_WIDTH_X_m / 2.0, -CFG.TRACK_WIDTH_Y_m / 2.0);
    return translations;
  }

  public void updateSimulationField() {
    SimulatedArena.getInstance().simulationPeriodic();

    Logger.recordOutput("Sim/Pose", sim.getSimulatedDriveTrainPose());

    final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
    if (notes != null) Logger.recordOutput("Sim/Notes", notes.toArray(Pose3d[]::new));
  }
}
