package frc.robot.auto;

public class RunAltAuto {

    private AltAuto auto;
    private boolean first = true;

    public RunAltAuto(AltAuto auto) {
        this.auto = auto;
    }

    public void periodic() {
        if(first) {
            auto.init();
            first = false;
        }
        auto.execute();
    }


    

}
