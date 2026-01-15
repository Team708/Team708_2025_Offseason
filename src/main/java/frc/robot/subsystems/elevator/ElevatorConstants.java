package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public static final int MOTOR_COUNT = 2;
  public static final double MOTOR_REDUCTION = 10.0;
  public static final double CARRAIGE_MASS_LBS = 5.0;
  public static final double EFFECTIVE_DRUM_RADIUS_INCHES = 0.75;
  public static final double MIN_HEIGHT_INCHES = 0.0;
  public static final double MAX_HEIGHT_INCHES = 80.0;
  public static final boolean SIMULATE_GRAVITY = true;
  public static final double STARTING_HEIGHT_INCHES = 0.0;
  public static final double SIM_UPDATE_INTERVAL = 0.02;
  public static final double KP = 0.55;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double MAX_VOLTAGE = 3.0;
  public static final double MAX_VOLTAGE_SIM = 12.0;
  public static final double MIN_CLOSED_LOOP_OUTPUT = -1 * (MAX_VOLTAGE / 12);
  public static final double MAX_CLOSED_LOOP_OUTPUT = (MAX_VOLTAGE / 12);
  public static final double MANUAL_ADJUST_INCHES = 2;
  public static final double DEADBAND = 1;
  public static final double ZEROING_VOLTAGE = -1.0;
  public static final int CAN_ID_MOTOR_1 = 21;
  public static final int CAN_ID_MOTOR_2 = 22;
  public static final int CURRENT_LIMIT = 80;
  public static final int SPROCKET_TEETH = 15;
  public static final double POSITION_FACTOR = 1.5185;
  public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60;

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
