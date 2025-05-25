package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;

public interface ElevatorCtrl {
  public void periodic();

  // Manual
  public default void manualAdjustPosition(double meters) {}

  // System
  public default void setTargetPosition(ElevatorTarget target) {}

  public default boolean atTargetPosition() {
    return true;
  }
}
