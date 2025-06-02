package frc.robot.commands;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;
import frc.robot.subsystems.moon.MoonCtrl;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class MoonCommands {
  private static final LoggedTunableNumber manualChangeMoonRadians =
      new LoggedTunableNumber("Elevator/ManualAdjustMoonRadians", kManualAdjustRadians);

  public static Command manualControl(Moon moon, DoubleSupplier joystickValue) {
    return Commands.run(
        () -> {
          MoonCtrl control = moon.getMoonCtrl();
          control.manualAdjust(
              manualChangeMoonRadians.getAsDouble() * -joystickValue.getAsDouble());
        },
        moon);
  }

  public static Command moveToPosition(Moon moon, MoonTarget target) {
    return Commands.run(
            () -> {
              moon.getMoonCtrl().setTargetPosition(target);
            },
            moon)
        .until(() -> moon.getMoonCtrl().atTargetPosition())
        .beforeStarting(
            () -> System.out.println("Moon: moveToPosition " + target.toString() + " started"));
  }
}
