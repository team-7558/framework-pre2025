package frc.robot.superstructure;

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


    // Override toString() for debugging and display
    @Override
    public String toString() {
        return String.format("SSPose [alpha=%.2f, beta=%.2f, h=%.2f, phi=%.2f]", alpha, beta, h, phi);
    }
}
