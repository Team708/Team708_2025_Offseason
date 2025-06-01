package frc.robot.subsystems.moon;

import frc.robot.subsystems.moon.MoonConstants.MoonTarget;

public interface MoonCtrl {
  public void periodic();

  // Manual
  public default void manualAdjust(double radians) {}

  public default void setVoltage(double voltage) {}

  public default boolean atTargetPosition() {
    return false;
  }

  public default void setTargetPosition(MoonTarget target) {}
}
