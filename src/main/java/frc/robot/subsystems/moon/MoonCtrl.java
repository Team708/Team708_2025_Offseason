package frc.robot.subsystems.moon;

public interface MoonCtrl {
  public void periodic();

  // Manual
  public default void manualAdjust(double radians) {}

  public default void setVoltage(double voltage) {}
}
