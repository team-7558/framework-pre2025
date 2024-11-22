package frc.robot.subsystems.pinkarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class pinkarmIOSim implements pinkarmIO {

    private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(1),
          45,
          20,
          0.5,
          5,
          10,
          false,
          5,
          null);

    private PIDController elev_posPid = new PIDController(0.0, 0.0, 0.0);
    private double elev_appliedVolts = 0.0;

    private final PIDController anglePID = new PIDController(0.0, 0.0, 0.0);

    private pinkarm2d mech = new pinkarm2d();


    @Override
    public void updateInputs(pinkarmInputs inputs) {
        // Update inputs based on current state
        sim.update(0.05);
        inputs.elev_posMeters = sim.getPositionMeters();
        inputs.elev_velMPS = sim.getVelocityMetersPerSecond();
        inputs.elev_volts = elev_appliedVolts;
        inputs.elev_currents = new double[] {sim.getCurrentDrawAmps(), sim.getCurrentDrawAmps()};




    }



    @Override
    public void setelevVoltage(double volts) {
        elev_appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void setAngle(double degrees) {
        
        anglePID.setSetpoint(degrees);
        mech.setAngle(degrees);
    }

    @Override
    public void setLength(double meters) {
        elev_posPid.setSetpoint(meters);
        setelevVoltage(elev_posPid.calculate(sim.getPositionMeters()));
        mech.setLength(meters);
    }



    @Override
    public void stop() {
        setelevVoltage(0.0);
    }

}




  