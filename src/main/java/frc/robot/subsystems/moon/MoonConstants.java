package frc.robot.subsystems.moon;

import edu.wpi.first.math.system.plant.DCMotor;

public class MoonConstants {
  public static final double kMaxVoltage = 6.0;
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMotorReduction = 4.0;
  public static final double kJKgMetersSquared = 0.025;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kManualAdjustRadians = (2 * Math.PI) / 50;
  public static final double kMaxRadians = Math.PI;
  public static final int kCanID = 31;
  public static final DCMotor kMotor = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 1);
  public static final int kCurrentLimit = 80;
  public static final double kEncoderPositionFactor = (2 * Math.PI) / kMotorReduction;
  public static final double kEncoderVelocityFactor = kEncoderPositionFactor / 60.0;
  public static final double kDeadband = 0.05;

  public enum MoonTarget {
    CORAL_LOW(0),
    CORAL_HIGH(0.3),
    ALGAE_LOW(1.7),
    ALGAE_HIGH(3.0);

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
