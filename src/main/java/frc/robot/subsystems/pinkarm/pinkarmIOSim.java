package frc.robot.subsystems.pinkarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class PinkarmIOSim implements PinkarmIO {

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(1),
          45, // Gear ratio
          20, // Mass (kg)
          0.5, // Pulley radius (m)
          Pinkarm.ELEV_MIN_HEIGHT_M,
          Pinkarm.ELEV_MAX_HEIGHT_M,
          false,
          5,
          null);

  private final DCMotorSim armSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), 45, 3.67); // Custom arm motor simulation

  private final PIDController elevatorPositionPID = new PIDController(20, 1, 4);
  private final PIDController armPositionPID = new PIDController(70, 1, 4);

  private double elevatorAppliedVolts = 0.0;
  private double armAppliedVolts = 0.0;

  @Override
  public void updateInputs(PinkarmInputs inputs) {
    // Update elevator simulation
    elevatorSim.update(0.02); // 20 ms update
    inputs.elev_posMeters = elevatorSim.getPositionMeters();
    inputs.elev_velMPS = elevatorSim.getVelocityMetersPerSecond();
    inputs.elev_volts = elevatorAppliedVolts;
    inputs.elev_currents =
        new double[] {
          elevatorSim.getCurrentDrawAmps(), elevatorSim.getCurrentDrawAmps()
        }; // Simulate multiple motor currents

    // Update arm simulation
    armSim.update(0.02); // 20 ms update
    inputs.arm_posDegrees = Units.radiansToDegrees(armSim.getAngularPositionRad());
    inputs.arm_velDegPS = Units.radiansToDegrees(armSim.getAngularVelocityRadPerSec());
    inputs.arm_volts = armAppliedVolts;
    inputs.arm_currents =
        new double[] {
          armSim.getCurrentDrawAmps(), armSim.getCurrentDrawAmps()
        }; // Simulate multiple motor currents
  }

  @Override
  public void setelevVoltage(double volts) {
    elevatorAppliedVolts = volts;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void setArmVoltage(double volts) {
    armAppliedVolts = volts;
    armSim.setInputVoltage(volts);
  }

  @Override
  public void goToAngle(double degrees) {
    // Check if the target is valid (optional safety check)
    double targetRadians = Units.degreesToRadians(degrees);
    armPositionPID.setSetpoint(targetRadians);
    double calculatedVoltage = armPositionPID.calculate(armSim.getAngularPositionRad());
    setArmVoltage(calculatedVoltage);
  }

  @Override
  public void goToPos(double meters) {
    elevatorPositionPID.setSetpoint(meters);
    double calculatedVoltage = elevatorPositionPID.calculate(elevatorSim.getPositionMeters());
    setelevVoltage(calculatedVoltage);
  }

  @Override
  public void stop() {
    setelevVoltage(0.0);
    setArmVoltage(0.0);
  }
}
