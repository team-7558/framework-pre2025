package frc.robot.subsystems.hangV2;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HangIOSim implements HangIO {
    DCMotorSim pivotSim = new DCMotorSim(DCMotor.getKrakenX60Foc(2), 1, 1);
    DCMotorSim rollSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), 1, 1);


    @Override
    public void updateInputs(HangIOInputs inputs) {
        pivotSim.update(0.02);
        rollSim.update(0.02);
        inputs.positionDeg = Units.radiansToDegrees(motorSim.getAngularPositionRad());
        inputs.currentAmps = new double[] {motorSim.getCurrentDrawAmps(), motorSim.getCurrentDrawAmps()};
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public void stop() {
        pivotSim.setInputVoltage(0.0);
        rollSim.setInputVoltage(0.0);
    }
}
