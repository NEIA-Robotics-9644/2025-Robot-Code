package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Set this to false during comp to lock in the tuning values
  public static boolean tuningMode = true;

  public static Map<String, Double> extenderAngles =
      Map.of(
          "L1", 25.0,
          "L2", 30.0,
          "L3", 35.0,
          "L4", 100.0,
          "Intake", 1.0,
          "Process", 2.0,
          "L2Dealgify", 3.0,
          "L3Dealgify", 4.0,
          "Zero", 0.0);
  public static Map<String, Double> effectorAngles =
      Map.of(
          "L1", 25.0,
          "L2", 30.0,
          "L3", 35.0,
          "L4", 100.0,
          "Intake", 1.0,
          "Process", 2.0,
          "L2Dealgify", 3.0,
          "L3Dealgify", 4.0,
          "Zero", 0.0);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
