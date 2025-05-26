package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
  public static final int kNumMotors = 2;
  public static final double kMotorReduction = 10.0;
  public static final double kCarriageMassKg = 4.0;
  public static final double kDrumRadiusMeters = 0.0152;
  public static final double kMinHeightMeters = 0.0;
  public static final double kMaxHeightMeters = 2.0;
  public static final boolean kSimulateGravity = true;
  public static final double kStartingHeightMeters = 0.0;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kP = 16.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMaxVoltage = 12.0;
  public static final double kManualAdjustMeters = 0.05;
  public static final DCMotor kMotors = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 2);
  public static final double kL0 = 0.0;
  public static final double kL1 = 0.45;
  public static final double kL2 = 0.9;
  public static final double kL3 = 1.3;
  public static final double kL4 = 1.8;
  public static final double kDeadband = 0.1;
  public static final double kZeroingVoltage = -1.0;
  public static final int kCanIDMotor1 = 21;
  public static final int kCanIDMotor2 = 22;
  public static final int kCurrentLimit = 60;
  public static final double kEncoderPositionFactor = 0.0;
  public static final double kEncoderVelocityFactor = 0.0;
  public static final int kSprocketTeeth = 15;
  public static final double kpitchDiameterInches = kSprocketTeeth * 0.25 / Math.PI;
  public static final double kEffectiveRadiusMeters = (kpitchDiameterInches / 2.0) * 0.0254;
  public static final double kPositionFactor =
      (2 * Math.PI * kEffectiveRadiusMeters) / kMotorReduction;
  public static final double kVelocityFactor = kPositionFactor / 60.0;

  public enum ElevatorTarget {
    ALGAE_L0(0.2),
    ALGAE_L1(0.4),
    ALGAE_L2(0.8),
    ALGAE_L3(1.2),
    ALGAE_L4(1.9),
    CORAL_L0(0.0),
    CORAL_L1(0.45),
    CORAL_L2(0.9),
    CORAL_L3(1.3),
    CORAL_L4(1.8);

    public final double heightMeters;

    ElevatorTarget(double heightMeters) {
      this.heightMeters = heightMeters;
    }

    public static ElevatorTarget fromHeight(double heightMeters) {
      for (ElevatorTarget position : values()) {
        if (Math.abs(position.heightMeters - heightMeters) < 1e-3) {
          return position;
        }
      }
      return null; // or throw if needed
    }
  }
}
