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
  public static final int kCanId = 31;
  public static final DCMotor kMotor = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 1);
}
