package frc.robot.subsystems.elevatorWithArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorWithArmIOSim implements ElevatorWithArmIO{
    private ElevatorSim sim = 
    new ElevatorSim(DCMotor.getKrakenX60Foc(4),
                    25,
                    40,
                    0.05,
                    0.5,
                    3,
                    false,
                    0.5);
    private PIDController pid = new PIDController(0, 0, 0);
    private double appliedVoltsElev;
    private double appliedVoltsArm;
    @Override
    public void updateInputs(ElevatorWithArmIOInputs inputs) {
        appliedVoltsElev =
            MathUtil.clamp(pid.calculate(sim.getPositionMeters()), -12.0, 12.0);
        sim.setInputVoltage(appliedVoltsElev);
    }


    public void setVoltageElev (double volts) {
        appliedVoltsElev = volts;
        sim.setInputVoltage(volts);
    }

    public void setVoltageArm (double volts) {
        appliedVoltsArm = volts;
    }

    @Override
    public void stop() {
      setVoltageElev(0.0);
      setVoltageArm(0.0);
    }
}