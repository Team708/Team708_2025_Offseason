package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlSystem extends ElevatorCtrlBase implements ElevatorCtrl {
  private double targetInches;

  @Override
  protected void init() {
    targetInches = 0.0;
    Logger.recordOutput("Elevator/TargetLevel", "Unknown");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // Logger.recordOutput("Elevator/IsAtTarget", atTargetPosition());

    // Zeroing logic
    if (!inputs.reverseLimitTriggered && targetInches <= 0) {
      targetInches -= 1;
    }

    inputs.targetInches = targetInches;
  }

  @Override
  public void setTargetPosition(ElevatorTarget target) {
    Logger.recordOutput("Elevator/TargetLevel", target.name());
    targetInches = target.heightInches;
  }

  @Override
  public boolean atTargetPosition() {
    return Math.abs(inputs.positionInches - targetInches) <= kDeadband;
  }
}
