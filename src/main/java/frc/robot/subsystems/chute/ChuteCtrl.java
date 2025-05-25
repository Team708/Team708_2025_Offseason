package frc.robot.subsystems.chute;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;

public interface ChuteCtrl {
  public void periodic();

  // System
  public default void extend() {}

  public default void retract() {}

  // Manual
  public default void setTargetPosition(ElevatorTarget target) {}

  public default void setVoltage(double volts) {}
}
