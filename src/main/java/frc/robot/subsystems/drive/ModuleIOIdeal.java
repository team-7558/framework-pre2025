package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ModuleIOIdeal implements ModuleIO {

  ModuleIOInputs outputs = new ModuleIOInputs();

  public ModuleIOIdeal() {
    outputs.driveCurrent_A = new double[] {};
    outputs.drivePos_r = 0.0;
    outputs.driveVel_mps = 0.0;
    outputs.driveVolts_V = 0.0;
    outputs.driveCurrent_A = new double[] {};
    outputs.turnAbsPos_Rot2d = new Rotation2d();
    outputs.turnPos_Rot2d = new Rotation2d();
    outputs.turnVel_rps = 0.0;
    outputs.turnVolts_V = 0.0;
    outputs.turnCurrent_A = new double[] {};
    outputs.odometryTimestamps_s = new double[] {};
    outputs.odometryDrivePos_r = new double[] {};
    outputs.odometryTurnPos_Rot2d = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveCurrent_A = outputs.driveCurrent_A;
    inputs.drivePos_r = outputs.drivePos_r;
    inputs.driveVel_mps = outputs.driveVel_mps;
    inputs.driveVolts_V = outputs.driveVolts_V;
    inputs.driveCurrent_A = outputs.driveCurrent_A;
    inputs.turnAbsPos_Rot2d = outputs.turnAbsPos_Rot2d;
    inputs.turnPos_Rot2d = outputs.turnPos_Rot2d;
    inputs.turnVel_rps = outputs.turnVel_rps;
    inputs.turnVolts_V = outputs.turnVolts_V;
    inputs.turnCurrent_A = outputs.turnCurrent_A;
    inputs.odometryTimestamps_s = outputs.odometryTimestamps_s;
    inputs.odometryDrivePos_r = outputs.odometryDrivePos_r;
    inputs.odometryTurnPos_Rot2d = outputs.odometryTurnPos_Rot2d;
  }

  @Override
  public void setDriveDC(double vel_mps) {
    outputs.driveVolts_V = vel_mps / Drive.CFG.MAX_LINEAR_VEL_mps * 12.0;
    setDriveVel(vel_mps);
  }

  @Override
  public void setDriveVel(double vel_mps) {
    outputs.driveVel_mps = vel_mps;
    double rps = Units.radiansToRotations(vel_mps / Drive.CFG.WHEEL_RADIUS_m);
    outputs.drivePos_r += rps * Constants.globalDelta_s;

    // TODO: fix logic
    int subticks =
        1; // (int) Math.ceil(Swerve.CFG.ODOMETRY_FREQUENCY_Hz / Constants.globalDelta_Hz);
    final double[] odometryTimestamps = new double[subticks];
    final double[] odometryDrivePos_r = new double[subticks];

    for (int i = 0; i < subticks; i++) {
      double factor = ((i + 1) / ((double) subticks) - 1);
      odometryTimestamps[i] = Timer.getFPGATimestamp(); // + Constants.globalDelta_s * factor;
      odometryDrivePos_r[i] = outputs.drivePos_r; // + rps * factor;
    }
    Logger.recordOutput("T", Timer.getFPGATimestamp());
    outputs.odometryTimestamps_s = odometryTimestamps;
    outputs.odometryDrivePos_r = odometryDrivePos_r;
  }

  @Override
  public void setTurnPos(double measured_r, double setpoint_r) {
    outputs.turnVel_rps = (setpoint_r - measured_r) * Constants.globalDelta_Hz;
    // System.out.println(Rotation2d.fromRotations(pos_r).getDegrees());
    outputs.turnPos_Rot2d = Rotation2d.fromRotations(setpoint_r);

    int subticks =
        1; // (int) Math.ceil(Swerve.CFG.ODOMETRY_FREQUENCY_Hz / Constants.globalDelta_Hz);
    final Rotation2d[] odometryTurnPos_Rot2d = new Rotation2d[subticks];
    for (int i = 0; i < subticks; i++) {
      double factor = ((i + 1) / ((double) subticks) - 1);
      odometryTurnPos_Rot2d[i] =
          Rotation2d.fromRotations(setpoint_r /*+ outputs.turnVel_rps * factor*/);
    }
    outputs.odometryTurnPos_Rot2d = odometryTurnPos_Rot2d;
  }

  @Override
  public void stop() {
    setDriveVel(0);
    setTurnVoltage(0);
  }

  @Override
  public void setBrake(boolean brake) {}
}
