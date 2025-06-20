package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlSystem extends SubsystemBase implements ElevatorCtrl {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private double targetInches;

  public ElevatorCtrlSystem(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
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
