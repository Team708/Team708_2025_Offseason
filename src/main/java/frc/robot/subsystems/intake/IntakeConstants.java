package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final double kMotorReduction = 4.0;
  public static final double kAlgaeOutakeVoltage = 3.0;
  public static final double kAlgaeIntakeVoltage = -3.0;
  public static final double kCoralIntakeVoltage = 1.3;
  public static final double kCoralOutakeVoltage = 2.0;
  public static final double kHoldingVoltage = 3.0;
  public static final double kJKgMetersSquared = 0.025;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final int kCanID = 32;
  public static final int kCurrentLimit = 80;
  public static final int kBeamChannel = 1;
  public static final double kEncoderPositionFactor = (2 * Math.PI) / kMotorReduction;
}
