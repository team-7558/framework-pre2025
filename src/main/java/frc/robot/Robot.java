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

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.subsystems.pinkarm.Claw;
import frc.robot.subsystems.pinkarm.ClawModes;
import frc.robot.subsystems.pinkarm.Pinkarm;
import frc.robot.subsystems.pinkarm.elevModes;
import frc.robot.superstructure.InternalState;
import frc.robot.superstructure.SS;
import frc.robot.util.Util;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Drive drive;
  private Pinkarm arm = Pinkarm.getInstance();
  private Claw claw = Claw.getInstance();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();

    // init subsystems
    drive = Drive.getInstance();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.

    double x_ = OI.deadband(-OI.DR.getLeftY());
    double y_ = OI.deadband(-OI.DR.getLeftX());
    double w_ = 1.0 * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
    double throttle = Util.sqInput(1.0 - OI.deadband(OI.DR.getLeftTriggerAxis()));
    boolean a_button = OI.DR.getAButton();
    boolean x_button = OI.DR.getXButton();
    boolean y_button = OI.DR.getYButton();
    boolean b_button = OI.DR.getBButton();

    SwerveInput input = new SwerveInput(x_, y_, w_, throttle);
    drive.setInput(input);

    SS.getInstance().handleStateMachine();
    drive.periodic();
    // swerve.periodic();
    PerfTracker.periodic();

    if (a_button) {
      claw.queueState(ClawModes.OPEN);
    } else if (x_button) {
      arm.queueState(elevModes.TRAVELLING);
      arm.PlaceEndEffector(7, 3);
    } else if (y_button) {
      arm.queueState(elevModes.TRAVELLING);
      arm.PlaceEndEffector(-5, 5);
    } else if (b_button) {
      SS.getInstance().queueState(InternalState.PICK_UP_CONE);
    } else if (!b_button) {
      arm.queueState(elevModes.TRAVELLING);
      arm.PlaceEndEffector(0, 5);
      SS.getInstance().queueState(InternalState.IDLE);

    } else {
      arm.queueState(elevModes.TRAVELLING);
      arm.PlaceEndEffector(0, 5);
    }

    CommandScheduler.getInstance().run();

    arm.periodic();
    claw.periodic();
    // ^ will be gone later just keeping now to not break shit
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    SS.getInstance().queueState(InternalState.DISABLED);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Runs at the start of auto */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // run auto!
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SS.getInstance().queueState(InternalState.IDLE);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    drive.updateSimulationField();
  }
}
