package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int kCanID = 61;
  public static final int kCurrentLimit = 40;
  public static final double kMotorReduction = 10.0;
  public static final double kJKgMetersSquared = 0.025;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kEncoderPositionFactor = 0.2;
  public static final double kEncoderVelocityFactor = kEncoderPositionFactor / 60.0;
  public static final double kMaxVoltage = 12.0;
  public static final double kZeroingVoltage = 8.0;
  public static final int kBeamBreak1 = 7;
  public static final int kBeamBreak2 = 9;
  public static final int kCageLimitSwitch1 = 2;
  public static final int kCageLimitSwitch2 = 3;
  public static final int kServoChannel = 9;
  public static final double kP = 6.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kExtendedRadians = Math.toRadians(90);
  public static final double kServoReleasePosition = 0.5;
  public static final double kServoBrakePosition = 0.75;

  public enum ClimberState {
    UNKNOWN,
    RETRACTED,
    EXTENDED,
    ENGAGED,
    CLIMBED
  }
}
