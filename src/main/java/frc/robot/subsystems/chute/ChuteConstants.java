package frc.robot.subsystems.chute;

public class ChuteConstants {
  public static final int kCanID = 51;
  public static final int kCurrentLimit = 40;
  public static final double kMotorReduction = 5;
  public static final double kScrewInchesPerRev = 0.2;
  public static final double kEncoderPositionFactor = 0.2;
  public static final double kEncoderVelocityFactor = kEncoderPositionFactor / 60.0;
  public static final double kP = 12.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kRetractedInches = 0.0;
  public static final double kExtendedInches = 13.0;
  public static final double kMassLbs = 1.0;
  public static final double kEffectiveRadius = 0.005;
  public static final double kTolerance = 0.005;
  public static final double kMaxVoltage = 6.0;
  public static final double kZeroingVoltage = 1.0;
}
