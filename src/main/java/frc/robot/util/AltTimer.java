package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class AltTimer implements ITimed {

  private boolean active;
  private Timer timer;

  public AltTimer() {
    timer = new Timer();
    timer.start();
    active = false;
  }

  public double time() {
    if (active) {
      return timer.get();
    } else {
      return 0.0;
    }
  }

  public void reset() {
    timer.reset();
    active = true;
  }

  public boolean after(double seconds) {
    if (active) {
      return timer.hasElapsed(seconds);
    } else {
      return false;
    }
  }

  public boolean before(double seconds) {
    if (active) {
      return !timer.hasElapsed(seconds);
    } else {
      return true;
    }
  }

  public boolean between(double earlier, double later) {
    if (active) {
      double t = timer.get();
      return earlier < t && t < later;
    } else {
      return false;
    }
  }

  public double alpha(double earlier, double later) {
    if (active) {
      double t = timer.get();
      if (t <= earlier) return 0.0;
      if (t >= later) return 1.0;
      return Util.unlerp(earlier, later, t);
    } else {
      return 0.0;
    }
  }

  public boolean active() {
    return active;
  }
}
