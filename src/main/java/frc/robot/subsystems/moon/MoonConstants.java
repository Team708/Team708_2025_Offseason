package frc.robot.subsystems.moon;

public class MoonConstants {
  public static final double MAX_VOLTAGE = 4.0;
  public static final double MIN_CLOSED_LOOP_OUTPUT = -1 * (MAX_VOLTAGE / 12);
  public static final double MAX_CLOSED_LOOP_OUTPUT = (MAX_VOLTAGE / 12);
  public static final double KP = 2.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double MOTOR_REDUCTION = 4.0;
  public static final double J_KG_METERS_SQUARED = 0.025;
  public static final double SIM_UPDATE_INTERVAL = 0.02;
  public static final double MANUAL_ADJUST_RADIANS = (2 * Math.PI) / 50;
  public static final double MAX_RADIANS = 2 * Math.PI;
  public static final int CAN_ID = 31;
  public static final int CURRENT_LIMIT = 80;
  public static final double ENCODER_POSITION_FACTOR = 0.0815;
  public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0;
  public static final double DEADBAND = 0.1;
  public static final double ZEROING_VOLTAGE = 2.0;

  public enum MoonTarget {
    CORAL_LOW(Math.toRadians(0)),
    CORAL_HIGH(Math.toRadians(15)),
    ALGAE_LOW(Math.toRadians(160)),
    ALGAE_HIGH(Math.toRadians(45));

    public final double radians;

    MoonTarget(double radians) {
      this.radians = radians;
    }

    public static MoonTarget fromRadians(double radians) {
      for (MoonTarget position : values()) {
        if (Math.abs(position.radians - radians) < 1e-3) {
          return position;
        }
      }
      return null; // or throw if needed
    }
  }
}
