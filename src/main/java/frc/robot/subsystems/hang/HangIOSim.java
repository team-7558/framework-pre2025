package frc.robot.subsystems.hang;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.util.AltTimer;

public class HangIOSim implements HangIO {

    private final AltTimer timer = new AltTimer();

    private TrapezoidProfile.State armGoal;
    private TrapezoidProfile.State armStartpoint;
    private TrapezoidProfile.State armSetpoint;

    private final TrapezoidProfile.Constraints armConstraints = new TrapezoidProfile.Constraints(100, 0);

    private final TrapezoidProfile armProfile = new TrapezoidProfile(armConstraints);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), 20, 4.5, 2, Units.degreesToRadians(0), Units.degreesToRadians(360), false, 0);
    private final PIDController armPositionPID = new PIDController(40, 1, 1);
    private double applied_volts = 0.0;

    @Override
    public void updateInputs(HangIOInputs inputs) {
        // update stuff
        armSim.update(0.02);
        inputs.pos_deg = Units.radiansToDegrees(armSim.getAngleRads());
        inputs.velDegPS = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
        inputs.volts = applied_volts;
        inputs.currentAmps = new double[] {armSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVolts(double volts) {
        applied_volts = volts;
        armSim.setInputVoltage(volts);
    }

    @Override
    public void setAngle(double degrees, HangIOInputs inputs) {
        timer.reset();
        armGoal = new TrapezoidProfile.State(degrees, 0);
        armStartpoint = new TrapezoidProfile.State(inputs.pos_deg, inputs.velDegPS);

        armSetpoint = armProfile.calculate(timer.time(), armStartpoint, armGoal);
        

        Logger.recordOutput("Hang/ArmSetPoint", armSetpoint.position);
    }
}
