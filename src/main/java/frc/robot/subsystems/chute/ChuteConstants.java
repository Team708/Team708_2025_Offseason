package frc.robot.subsystems.chute;

public class ChuteConstants {
  public static final int CAN_ID = 51;
  public static final int CURRENT_LIMIT = 40;
  public static final double MOTOR_REDUCTION = 5;
  public static final double SCREW_INCHES_PER_REV = 0.2;
  public static final double ENCODER_POSITION_FACTOR = 0.2;
  public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0;
  public static final double KP = 12.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double SIM_UPDATE_INTERVAL = 0.02;
  public static final double RETRACTED_INCHES = 0.0;
  public static final double EXTENDED_INCHES = 13.0;
  public static final double MASS_LBS = 1.0;
  public static final double EFFECTIVE_RADIUS = 0.005;
  public static final double TOLERANCE = 0.005;
  public static final double MAX_VOLTAGE = 6.0;
  public static final double ZEROING_VOLTAGE = 1.0;
}
