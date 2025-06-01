package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {
  public static final double kMotorReduction = 4.0;
  public static final double kAlgaeOutakeVoltage = 3.0;
  public static final double kAlgaeIntakeVoltage = 3.0;
  public static final double kCoralIntakeVoltage = 3.0;
  public static final double kCoralOutakeVoltage = 3.0;
  public static final double kHoldingVoltage = 3.0;
  public static final double kJKgMetersSquared = 0.025;
  public static final double kSimUpdateInterval = 0.02;
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final DCMotor kMotor = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 1);
}
