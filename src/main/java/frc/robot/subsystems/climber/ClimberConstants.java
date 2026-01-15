package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int CAN_ID = 61;
  public static final int CURRENT_LIMIT = 40;
  public static final double MOTOR_REDUCTION = 45.0;
  public static final double J_KG_METERS_SQUARED = 0.025;
  public static final double SIM_UPDATE_INTERVAL = 0.02;
  public static final double ENCODER_POSITION_FACTOR = .00624;
  public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0;
  public static final double MAX_VOLTAGE = 6.0;
  public static final double ZEROING_VOLTAGE = -6.0;
  public static final int BEAM_BREAK_1_CHANNEL = 7;
  public static final int BEAM_BRAKE_2_CHANNEL = 9;
  public static final int CAGE_LIMIT_SWITCH_1_CHANNEL = 2;
  public static final int CAGE_LIMIT_SWITCH_2_CHANNEL = 3;
  public static final int SERVO_CHANNEL = 9;
  public static final double KP = 8.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double EXTENDED_RADIANS = Math.toRadians(50.0);
  public static final double SERVO_RELEASE_POSITION = 0.5;
  public static final double SERVO_BRAKE_POSITION = 0.75;
  public static final double DEADBAND = Math.toRadians(1);

  public enum ClimberState {
    UNKNOWN,
    RETRACTED,
    EXTENDED,
    ENGAGED,
    CLIMBED
  }
}
