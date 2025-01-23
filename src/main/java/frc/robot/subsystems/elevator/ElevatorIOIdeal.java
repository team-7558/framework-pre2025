package frc.robot.subsystems.elevator;

public class ElevatorIOIdeal implements ElevatorIO {

  public double pos_m = 0.0;
  public double vel_mps = 0.0;
  public double volts_V = 0.0;
  public double[] currents_A = new double[] {};

  public boolean hallEffect = false;

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.currents_A = currents_A;
    inputs.pos_m = pos_m;
    inputs.vel_mps = vel_mps;
    inputs.volts_V = volts_V;


  }

    public void setPos_m(double pos_m) {
        this.pos_m = pos_m;
    }

    public void setVel_mps(double vel_mps) {
        this.vel_mps = vel_mps;
    }

    public void setVolts_V(double volts_V) {
        this.volts_V = volts_V;
    }

    public void setCurrents_A(double[] currents_A) {
        this.currents_A = currents_A;
    }

    public void setHallEffect(boolean hallEffect) {
        this.hallEffect = hallEffect;
    }
}
