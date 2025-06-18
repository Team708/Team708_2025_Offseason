package frc.robot.subsystems.climber;

public interface ClimberCtrl {
  public void periodic();

  public default void startClimb() {}

  public default void setVoltage(double voltage) {}

  public default void setServo(boolean isUnlocked) {}
}
