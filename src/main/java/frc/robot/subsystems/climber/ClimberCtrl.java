package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimberConstants.ClimberState;

public interface ClimberCtrl {
  public void periodic();

  public default void startClimb() {}

  public default void setVoltage(double voltage) {}

  public default void setServo(boolean isUnlocked) {}

  public default boolean isAtDesiredState() {
    return true;
  }

  public default boolean readyToClimb() {
    return false;
  }

  public default ClimberState getClimberState() {
    return ClimberState.UNKNOWN;
  }

  public default ClimberState getDesiredClimberState() {
    return ClimberState.UNKNOWN;
  }
}
