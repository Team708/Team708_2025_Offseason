package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberCtrl;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
  private static final LoggedTunableNumber manualVolts =
      new LoggedTunableNumber("Climber/ManualVolts", 12.0);

  public static Command manualControl(Climber climber, DoubleSupplier joystickValue) {
    return Commands.run(
        () -> {
          ClimberCtrl control = climber.getClimberCtrl();
          Double voltage = joystickValue.getAsDouble() * manualVolts.getAsDouble();
          control.setVoltage(voltage);
        },
        climber);
  }

  public static Command deployClimber(Climber climber) {
    return Commands.run(
            () -> {
              ClimberCtrl control = climber.getClimberCtrl();
              control.startClimb();
            },
            climber)
        .until(() -> climber.getClimberCtrl().isAtDesiredState())
        .beforeStarting(() -> System.out.println("Climber: deployClimber started"));
  }
}
