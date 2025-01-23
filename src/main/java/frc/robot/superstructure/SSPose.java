package frc.robot.superstructure;

import frc.robot.Constants;

public class SSPose {

    private double alpha; //alpha is elbow absolute angle from horizontal
    private double beta;  //beta is shoulder absolute angle from horizontal
    private double h;  //h is height of the elevator from the ground to arm center pivot point
    private double phi;  //phi is the absolute angle of tongue from horizontal

    public SSPose(double alpha, double beta, double h, double phi) {
        this.alpha = alpha;
        this.beta = beta;
        this.h = h;
        this.phi = phi;
    }


    public double getAlpha() {
        return alpha;
    }

    public double getBeta() {
        return beta;
    }

    public double getH() {
        return h;
    }

    public double getPhi() {
        return phi;
    }


    public void copy(double Alpha, double Beta, double H, double Phi) {
        this.alpha = Alpha;
        this.beta = Beta;
        this.h = H;
        this.phi = Phi;
    }

    public void copy(SSPose pose) {
        this.alpha = pose.getAlpha();
        this.beta = pose.getBeta();
        this.h = pose.getH();
        this.phi = pose.getPhi();
    }


    public EEPose getEEPose() {
        double x = Constants.armlength * Math.cos(this.alpha);
        double y = Constants.armlength * Math.sin(this.alpha) * Math.cos(this.beta) + Math.cos(this.beta) * Constants.armoffset_in;
        double z = Constants.armlength * Math.sin(this.alpha) * Math.sin(this.beta) + Constants.elevheighttoarm - Math.sin(this.beta) * Constants.armoffset_in;

        double roll = 0.0;
        double angle_from_horizontal = Math.asin(Constants.armlength * Math.sin(this.alpha) * Math.sin(this.beta) - Constants.armoffset_in * Math.sin(this.beta));

        return new EEPose(x, y, z, roll, angle_from_horizontal);
    }
    // Override toString() for debugging and display
    @Override
    public String toString() {
        return String.format("SSPose [alpha=%.2f, beta=%.2f, h=%.2f, phi=%.2f]", alpha, beta, h, phi);
    }
}
