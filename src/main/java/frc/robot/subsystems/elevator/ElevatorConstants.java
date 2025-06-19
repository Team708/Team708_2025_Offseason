package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public static final int kNumMotors = 2;
  public static final double kMotorReduction = 10.0;
  public static final double kCarriageMassLbs = 5.0;
  public static final double kEffectiveDrumRadiusInches = 0.75;
  public static final double kMinHeightInches = 0.0;
  public static final double kMaxHeightInches = 80.0;
  public static final boolean kSimulateGravity = true;
  public static final double kStartingHeightInches = 0.0;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kP = 0.55;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMaxVoltage = 3.0;
  public static final double kMinClosedLoopOutput = -1 * (kMaxVoltage / 12);
  public static final double kMaxClosedLoopOutput = (kMaxVoltage / 12);
  public static final double kManualAdjustInches = 2;
  public static final double kDeadband = 1;
  public static final double kZeroingVoltage = -1.0;
  public static final int kCanIDMotor1 = 21;
  public static final int kCanIDMotor2 = 22;
  public static final int kCurrentLimit = 80;
  public static final int kSprocketTeeth = 15;
  public static final double kPositionFactor = 1.5185;
  public static final double kVelocityFactor = kPositionFactor / 60;

  public enum ElevatorLevel {
    L0,
    L1,
    L2,
    L3,
    L4
  }

  public enum ElevatorTarget {
    ALGAE_L0(8),
    ALGAE_L1(16),
    ALGAE_L2(32),
    ALGAE_L3(48),
    ALGAE_L4(74),
    CORAL_L0(0.0),
    CORAL_L1(18),
    CORAL_L2(35),
    CORAL_L3(51),
    CORAL_L4(79);

    public final double heightInches;

    ElevatorTarget(double heightInches) {
      this.heightInches = heightInches;
    }

    public static ElevatorTarget fromHeight(double heightInches) {
      for (ElevatorTarget position : values()) {
        if (Math.abs(position.heightInches - heightInches) < 1e-3) {
          return position;
        }
      }
      return null; // or throw if needed
    }
  }
}
