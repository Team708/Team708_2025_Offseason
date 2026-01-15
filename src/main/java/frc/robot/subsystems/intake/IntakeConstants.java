package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final double MOTOR_REDUCTION = 4.0;
  public static final double ALGAE_OUTTAKE_VOLTAGE = 3.0;
  public static final double ALGAE_INTAKE_VOLTAGE = -3.0;
  public static final double CORAL_INTAKE_VOLTAGE = 1.3;
  public static final double CORAL_OUTTAKE_VOLTAGE = 2.0;
  public static final double HOLDING_VOLTAGE = 3.0;
  public static final double J_KG_METERS_SQUARED = 0.025;
  public static final double SIM_UPDATE_INTERVAL = 0.02;
  public static final double KP = 1.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final int CAN_ID = 32;
  public static final int CURRENT_LIMIT = 80;
  public static final int BEAM_CHANNEL = 1;
  public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / MOTOR_REDUCTION;
}
