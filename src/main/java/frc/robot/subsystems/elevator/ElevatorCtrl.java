package frc.robot.subsystems.elevator;

public interface ElevatorCtrl {
  public void periodic();

  // Manual
  public default void setVoltage(double volts) {}
}
