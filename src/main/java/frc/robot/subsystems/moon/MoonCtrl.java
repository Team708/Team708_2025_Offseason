package frc.robot.subsystems.moon;

public interface MoonCtrl {
  public void periodic();

  public default void setVoltage(double voltage) {}
}
