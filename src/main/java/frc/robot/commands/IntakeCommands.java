package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeCtrlSystem.IntakeMode;

public class IntakeCommands {
  public static Command setMode(Intake intake, IntakeMode mode) {
    return Commands.runOnce(
            () -> {
              IntakeCtrlSystem control = intake.getIntakeCtrl();
              control.setMode(mode);
              control.disableHold();
            },
            intake)
        .beforeStarting(() -> System.out.println("Intake: setMode started"));
  }

  public static Command intakeCoral(Intake intake) {
    return Commands.run(
            () -> {
              IntakeCtrlSystem control = intake.getIntakeCtrl();
              control.disableHold();
              control.setMode(IntakeMode.CORAL_INTAKE);
              System.out.println("Running loop intakeCoral");
            },
            intake)
        .finallyDo(() -> intake.getIntakeCtrl().holdCurrentPosition())
        .until(() -> intake.getIntakeCtrl().hasCoral())
        .beforeStarting(() -> System.out.println("Intake: intakeCoral started"));
  }

  public static Command outakeCoral(Intake intake) {
    return Commands.run(
            () -> {
              IntakeCtrlSystem control = intake.getIntakeCtrl();
              control.setMode(IntakeMode.CORAL_OUTAKE);
              control.disableHold();
            },
            intake)
        .until(() -> !intake.getIntakeCtrl().hasCoral())
        .finallyDo(() -> intake.getIntakeCtrl().setMode(IntakeMode.STOP))
        .beforeStarting(() -> System.out.println("Intake: outakeCoral started"));
  }

  public static Command intakeAlgae(Intake intake) {
    return Commands.run(
            () -> {
              IntakeCtrlSystem control = intake.getIntakeCtrl();
              control.disableHold();
              control.setMode(IntakeMode.ALGAE_INTAKE);
            },
            intake)
        .until(() -> intake.getIntakeCtrl().hasAlgae())
        .finallyDo(() -> intake.getIntakeCtrl().holdCurrentPosition())
        .beforeStarting(() -> System.out.println("Intake: intakeAlgae started"));
  }

  public static Command outtakeAlgae(Intake intake) {
    return Commands.run(
            () -> {
              IntakeCtrlSystem control = intake.getIntakeCtrl();
              control.setMode(IntakeMode.CORAL_OUTAKE);
              control.disableHold();
            },
            intake)
        .withTimeout(1.5)
        .finallyDo(() -> intake.getIntakeCtrl().setMode(IntakeMode.STOP))
        .beforeStarting(() -> System.out.println("Intake: outtakeAlgae started"));
  }
}
