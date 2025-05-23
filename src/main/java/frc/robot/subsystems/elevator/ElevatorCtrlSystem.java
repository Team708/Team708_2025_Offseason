package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.kMaxVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ElevatorCtrlSystem extends SubsystemBase implements ElevatorCtrl {
  public enum Position {
    L0,
    L1,
    L2,
    L3,
    L4
  }

  private LoggedTunableNumber maxVoltage =
      new LoggedTunableNumber("Elevator/MaxVoltage", kMaxVoltage);
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs;
  private double targetMeters;

  public ElevatorCtrlSystem(ElevatorIO io) {
    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();
    targetMeters = 0.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
