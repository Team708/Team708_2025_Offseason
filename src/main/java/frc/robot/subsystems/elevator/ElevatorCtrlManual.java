package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlManual extends SubsystemBase implements ElevatorCtrl {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;

  public ElevatorCtrlManual(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
