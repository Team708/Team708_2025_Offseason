package frc.robot.subsystems.moon;

public class MoonConstants {
  public static final double kMaxVoltage = 4.0;
  public static final double kMinClosedLoopOutput = -1 * (kMaxVoltage / 12);
  public static final double kMaxClosedLoopOutput = (kMaxVoltage / 12);
  public static final double kP = 2.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMotorReduction = 4.0;
  public static final double kJKgMetersSquared = 0.025;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kManualAdjustRadians = (2 * Math.PI) / 50;
  public static final double kMaxRadians = 2 * Math.PI;
  public static final int kCanID = 31;
  public static final int kCurrentLimit = 80;
  public static final double kEncoderPositionFactor = 0.0815;
  public static final double kEncoderVelocityFactor = kEncoderPositionFactor / 60.0;
  public static final double kDeadband = 0.1;
  public static final double kZeroingVoltage = 2.0;

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
