package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SS;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Util;

public abstract class AltAuto {

  private String name;

  public final Trajstack trajstack;

  protected final Drive drive;
  protected final SS ss;
  
  private HashMap<BooleanSupplier,Runnable> callbacks = new HashMap<>();

  private Timer t;
  private boolean forcePoseReset;

  private boolean generated = false;

  public AltAuto(String s, boolean forcePoseReset) {
    name = s;
    drive = Drive.getInstance();
    ss = SS.getInstance();
    trajstack = new Trajstack();
    this.forcePoseReset = forcePoseReset;
    t = new Timer();
  }

  protected abstract void onInit();

  protected abstract void onExecute();

  /** Use this method to register any custom conditions and their associated actions */
  protected void registerCallback(BooleanSupplier condition, Runnable action) {
    this.callbacks.put(condition, action);
  }

  private void executeCallbacks() {
    ArrayList<BooleanSupplier> shouldRemove = new ArrayList<>();
    for(Entry<BooleanSupplier,Runnable> callback : callbacks.entrySet()) {
        if(callback.getKey().getAsBoolean()) {
            callback.getValue().run();
            shouldRemove.add(callback.getKey());
        }
    }
    shouldRemove.forEach(callback -> {
        callbacks.remove(callback); // this might cause concurrent mod exception lol mb
    });
  }


  public final void init() {
    System.out.println("Starting " + name);
    if (!generated) {
      generate();
    }

    if (forcePoseReset) {
      drive.setPose(new Pose2d(trajstack.getInitState().positionMeters, drive.getRotation()));
    }

    drive.setCurrentState(drive.PATHING);

    t.reset();
    t.start();
    onInit();
  }

  @Override
  public String toString() {
    return name;
  }

  public void generate() {
    trajstack.generate();
    generated = true;
  }

  public final void execute() {
    onExecute();
    executeCallbacks();
    double time = t.get();
    // if (time < 15.0) led.drawPreciseNumber(time, 16, 16, 16);
    // else led.drawNumber(time, 48, 0, 0);
    // TODO: ADD LED BACK 
  }

  protected boolean before(double time_s) {
    return t.get() < time_s;
  }

  protected boolean after(double time_s) {
    return t.get() > time_s;
  }

  protected boolean between(double time0_s, double time1_s) {
    return after(time0_s) && before(time1_s);
  }

  protected double alpha(double time0_s, double time1_s) {
    return Util.unlerp(time0_s, time1_s, t.get());
  }

  protected boolean near(double x, double y, double tol) {
    double dx = x - drive.getPose().getX();
    double dy = y - drive.getPose().getY();
    return dx * dx + dy * dy < tol * tol;
  }

  protected boolean velUnder(double mag) {
    // return drive.velUnder(mag);
    //todo: add
    return true;
  }

  protected double segEnd(int i) {
    return trajstack.segEnd(i);
  }
}