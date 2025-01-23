package frc.robot.superstructure;

public class EEPose {

    private double x; // X-coordinate
    private double y; // Y-coordinate
    private double z; // Z-coordinate (height or depth)
    private double roll; // Roll (rotation around the X-axis)
    private double angle_from_horizontal; // Absolute angle from horizontal

    public EEPose(double x, double y, double z, double roll, double angle_from_horizontal) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.angle_from_horizontal = angle_from_horizontal;
    }

    // Getters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getRoll() {
        return roll;
    }

    public double getAngleFromHorizontal() {
        return angle_from_horizontal;
    }


    // Copy methods
    public void copy(double x, double y, double z, double roll, double angle_from_horizontal) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.angle_from_horizontal = angle_from_horizontal;
    }

    public void copy(EEPose pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.z = pose.getZ();
        this.roll = pose.getRoll();
        this.angle_from_horizontal = pose.getAngleFromHorizontal();
    }

    // Override toString() for debugging and display
    @Override
    public String toString() {
        return String.format("EEPose [x=%.2f, y=%.2f, z=%.2f, roll=%.2f, angle_from_horizontal=%.2f]", 
                              x, y, z, roll, angle_from_horizontal);
    }
}
