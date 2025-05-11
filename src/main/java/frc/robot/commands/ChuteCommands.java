package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.chute.Chute;
import frc.robot.util.LoggedTunableNumber;

public class ChuteCommands {
  private static final LoggedTunableNumber maxOverrideVolts =
      new LoggedTunableNumber("Chute/MaxOverrideVolts", 12.0);
    private ChuteCommands() {}

    public static Command manualControl(Chute chute, DoubleSupplier magnitude) {
        return Commands.run(() -> {
            double overrideVoltage = magnitude.getAsDouble() * maxOverrideVolts.getAsDouble();
            chute.setManualOverride(true);
            chute.setManualVoltage(overrideVoltage);
        },
        chute)
                .finallyDo(() -> {
                    chute.setManualOverride(false);
                    chute.setManualVoltage(0.0);
                });
    }
}
