package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmIOIdeal implements ArmIO {
  private double ElbowVel_degps = 0.0;
  private double ElbowVolts_V = 0.0;
  private double ElbowPos_deg = 0.0;

  public ArmIOIdeal() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.elbow_volts = ElbowVolts_V;
    inputs.elbow_velDegPS = ElbowVel_degps;
    inputs.elbow_pos_deg = ElbowPos_deg;
    inputs.elbow_currents = new double[] {40, 40};
  }

  @Override
  public void setVelocity(double velocity_mps) {
    this.vel_mps = velocity_mps;
    this.pos_m += velocity_mps * Constants.globalDelta_sec;
  }

  @Override
  public void setVoltage(double volts_V) {
    this.volts_V = volts_V;
  }

  @Override
  public void holdPos(double pos_m) {
    this.vel_mps = 0;
    this.volts_V = 0;
    this.pos_m = pos_m;
  }

  @Override
  public void travelToPos(double pos_m) {
    this.vel_mps = 0;
    this.volts_V = 0;
    this.pos_m = pos_m;
  }
}