package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
  public static final int kNumMotors = 2;
  public static final double kMotorReduction = 10.0;
  public static final double kCarriageMassKg = 4.0;
  public static final double kDrumRadiusMeters = 0.0152;
  public static final double kMinHeightMeters = 0.0;
  public static final double kMaxHeightMeters = 1.5;
  public static final boolean kSimulateGravity = true;
  public static final double kStartingHeightMeters = 0.0;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kP = 8;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kMaxVoltage = 6.0;
  public static final DCMotor kMotors = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 2);
}
