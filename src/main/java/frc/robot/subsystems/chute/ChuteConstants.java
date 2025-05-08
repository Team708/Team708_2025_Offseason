package frc.robot.subsystems.chute;

public class ChuteConstants {
  public static final int kCanID = 51;
  public static final int kCurrentLimit = 40;
  private static final double kGearRatio = 10.0;
  public static final double kScrewTravelPerRev = 0.008;
  public static final double kEncoderPositionFactor = kGearRatio / kScrewTravelPerRev;
  public static final double kEncoderVelocityFactor = (kGearRatio / kScrewTravelPerRev) / 60.0;
  public static final double kMotorReduction = 1;
  public static final double kP = 50.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kRetractedMeters = 0.0;
  public static final double kExtendedMeters = 1;
  public static final double kTolerance = 0.005;
}
