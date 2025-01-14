package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {

  private TrapezoidProfile.State ElbowArmSetpoint;
  private TrapezoidProfile.State ElbowArmStartPoint;
  private TrapezoidProfile.State ElbowArmGoal;

  private final TrapezoidProfile.Constraints ElbowArmConstraints =
      new TrapezoidProfile.Constraints(90, 45);

  private final TrapezoidProfile ElbowArmProfile = new TrapezoidProfile(ElbowArmConstraints);

  private final AltTimer timer = new AltTimer();

  private final SingleJointedArmSim ElbowArmSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          45,
          3.67,
          0.5,
          Units.degreesToRadians(0),
          Units.degreesToRadians(90),
          false,
          Units.degreesToRadians(0)); // Custom arm motor simulation

  private final PIDController ElbowArmPositionPID = new PIDController(70, 1, 4);
  private double applied_volts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Update elevator simulation
    ElbowArmSim.update(0.02); // 20 ms update
    inputs.elbow_pos_deg = Units.radiansToDegrees(ElbowArmSim.getAngleRads());
    inputs.elbow_velDegPS = Units.radiansToDegrees(ElbowArmSim.getVelocityRadPerSec());
    inputs.elbow_volts = applied_volts;
    inputs.elbow_currents =
        new double[] {
          ElbowArmSim.getCurrentDrawAmps(), ElbowArmSim.getCurrentDrawAmps()
        }; // Simulate multiple motor left_currents
  }

  @Override
  public void setElbowVoltage(double left_volts) {
    applied_volts = left_volts;
    ElbowArmSim.setInputVoltage(left_volts);
  }

  @Override
  public void goToElbowAngle(double degrees, ArmIOInputs inputs, boolean first_time) {
    if (first_time) {
      System.out.println("Travelling once");
      timer.reset();
      ElbowArmGoal = new TrapezoidProfile.State(degrees, 0);
      ElbowArmStartPoint = new TrapezoidProfile.State(inputs.elbow_pos_deg, inputs.elbow_velDegPS);
    }

    ElbowArmSetpoint = ElbowArmProfile.calculate(timer.time(), ElbowArmStartPoint, ElbowArmGoal);
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(ElbowArmSetpoint.position);
    ElbowArmPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = ElbowArmPositionPID.calculate(ElbowArmSim.getAngleRads());
    setElbowVoltage(calculatedVoltage);

    Logger.recordOutput("Arm/ArmSetpoint", ElbowArmSetpoint.position);
  }

  @Override
  public void stopElbow() {
    setElbowVoltage(0.0);
  }
}
