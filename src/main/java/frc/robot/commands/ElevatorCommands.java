package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCtrl;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  private static final LoggedTunableNumber manualSpeedMeters =
      new LoggedTunableNumber("Elevator/ManualChangeMeters", 0.0254);

  private ElevatorCommands() {}

  public static Command manualControl(Elevator elevator, DoubleSupplier joystickValue) {
    return Commands.run(
        () -> {
          ElevatorCtrl control = elevator.getElevatorCtrl();
          control.changeElevatorPosition(
              manualSpeedMeters.getAsDouble() * joystickValue.getAsDouble());
        },
        elevator);
  }
}
