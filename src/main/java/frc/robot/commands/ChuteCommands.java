package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteCtrl;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class ChuteCommands {
  private static final LoggedTunableNumber manualVolts =
      new LoggedTunableNumber("Chute/ManualVolts", 6.0);

  private ChuteCommands() {}

  public static Command manualControl(Chute chute, DoubleSupplier joystickValue) {
    return Commands.run(
        () -> {
          ChuteCtrl control = chute.getChuteCtrl();
          Double voltage = joystickValue.getAsDouble() * manualVolts.getAsDouble();
          control.setVoltage(voltage);
        },
        chute);
  }

  public static Command extend(Chute chute) {
    return Commands.run(
            () -> {
              ChuteCtrl control = chute.getChuteCtrl();
              control.extend();
            },
            chute)
        .until(() -> chute.getChuteCtrl().isExtended())
        .beforeStarting(() -> System.out.println("Chute: extend starting"));
  }

  public static Command retract(Chute chute) {
    return Commands.run(
        () -> {
          ChuteCtrl control = chute.getChuteCtrl();
          control.retract();
        },
        chute)
        .until(()->chute.getChuteCtrl().isRetracted())
        .beforeStarting(()->System.out.println("Chute: retract starting"));
  }
}
