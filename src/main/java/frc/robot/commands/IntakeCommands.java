package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeCtrlSystem.Mode;

public class IntakeCommands {
  public static Command setMode(Intake intake, Mode mode) {
    return Commands.runOnce(
        () -> {
          IntakeCtrlSystem control = intake.getIntakeCtrl();
          control.setMode(mode);
          control.disableHold();
        },
        intake);
  }

  public static Command setHold(Intake intake, boolean hold) {
    return Commands.runOnce(
        () -> {
          IntakeCtrlSystem control = intake.getIntakeCtrl();
          if (hold) {
            control.holdCurrentPosition();
          } else {
            control.setMode(Mode.STOP);
            control.disableHold();
          }
        },
        intake);
  }
}
