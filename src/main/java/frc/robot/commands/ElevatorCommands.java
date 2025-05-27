package frc.robot.commands;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCtrl;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  private static final LoggedTunableNumber manualChangeMeters =
      new LoggedTunableNumber("Elevator/ManualAdjustMeters", kManualAdjustInches);

  private ElevatorCommands() {}

  public static Command manualControl(Elevator elevator, DoubleSupplier joystickValue) {
    return Commands.run(
        () -> {
          ElevatorCtrl control = elevator.getElevatorCtrl();
          control.manualAdjustPosition(
              manualChangeMeters.getAsDouble() * -joystickValue.getAsDouble());
        },
        elevator);
  }

  public static Command moveToLevel(Elevator elevator, ElevatorTarget target) {
    return Commands.run(
            () -> {
              elevator.getElevatorCtrl().setTargetPosition(target);
            },
            elevator)
        .until(() -> elevator.getElevatorCtrl().atTargetPosition());
  }
}
