package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;

public class CompositeCommands {
  public static Command changeMode(Elevator elevator, Moon moon) {
    if (moon.getMoonCtrl().getIsCoralMode()) {
      return Commands.sequence(
          moveToLevel(elevator, moon, ElevatorLevel.L2),
          MoonCommands.moveToTarget(moon, MoonTarget.ALGAE_LOW),
          new InstantCommand(
              () -> {
                moon.getMoonCtrl().setIsCoralMode(false);
              }));
    } else {
      return Commands.sequence(
          moveToLevel(elevator, moon, ElevatorLevel.L2),
          MoonCommands.moveToTarget(moon, MoonTarget.CORAL_LOW),
          new InstantCommand(
              () -> {
                moon.getMoonCtrl().setIsCoralMode(true);
              }));
    }
  }

  public static Command moveToLevel(Elevator elevator, Moon moon, ElevatorLevel level) {
    ElevatorTarget target;
    switch (level) {
      case L0:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L0 : ElevatorTarget.ALGAE_L0;
        break;
      case L1:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L1 : ElevatorTarget.ALGAE_L1;
        break;
      case L2:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L2 : ElevatorTarget.ALGAE_L2;
        break;
      case L3:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L3 : ElevatorTarget.ALGAE_L3;
        break;
      case L4:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L4 : ElevatorTarget.ALGAE_L4;
        break;
      default:
        target =
            moon.getMoonCtrl().getIsCoralMode() ? ElevatorTarget.CORAL_L0 : ElevatorTarget.ALGAE_L0;
        break;
    }
    if (moon.getMoonCtrl().getIsCoralMode()) {
      if (target == ElevatorTarget.CORAL_L4) {
        return Commands.sequence(
            ElevatorCommands.moveToTarget(elevator, target),
            MoonCommands.moveToTarget(moon, MoonTarget.CORAL_HIGH));
      } else {
        return Commands.sequence(
            MoonCommands.moveToTarget(moon, MoonTarget.CORAL_LOW),
            ElevatorCommands.moveToTarget(elevator, target));
      }
    } else {
      if (target == ElevatorTarget.ALGAE_L4) {
        return Commands.sequence(
            ElevatorCommands.moveToTarget(elevator, target),
            MoonCommands.moveToTarget(moon, MoonTarget.ALGAE_HIGH));
      } else {
        return Commands.sequence(
            MoonCommands.moveToTarget(moon, MoonTarget.ALGAE_LOW),
            ElevatorCommands.moveToTarget(elevator, target));
      }
    }
  }

  public static Command scoreCoral(
      Drive drive, Elevator elevator, Moon moon, Intake intake, int LocationID) {
    if (!moon.getMoonCtrl().getIsCoralMode() || !intake.getIntakeCtrl().hasCoral()) {
      return new InstantCommand();
    }
    Pose2d targetPose;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      targetPose = DriveConstants.poseMapBlue.get(LocationID);
    } else {
      targetPose = DriveConstants.poseMapRed.get(LocationID);
    }
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.driveToPose(targetPose, drive),
            moveToLevel(elevator, moon, ElevatorLevel.L4)),
        IntakeCommands.outakeCoral(intake),
        MoonCommands.moveToTarget(moon, MoonTarget.CORAL_LOW),
        ElevatorCommands.moveToTarget(elevator, ElevatorTarget.CORAL_L0));
  }

  public static Command climb(Elevator elevator, Moon moon, Chute chute, Climber climber) {
    return Commands.sequence(
            moveToLevel(elevator, moon, ElevatorLevel.L0),
            MoonCommands.moveToTarget(moon, MoonTarget.CORAL_HIGH),
            ChuteCommands.extend(chute),
            ClimberCommands.deployClimber(climber))
        .onlyIf(
            () -> moon.getMoonCtrl().getIsCoralMode() && climber.getClimberCtrl().readyToClimb());
  }

  public static Command resetRobot(Elevator elevator, Moon moon, Chute chute, Climber climber) {
    return Commands.sequence(
            moveToLevel(elevator, moon, ElevatorLevel.L0),
            MoonCommands.moveToTarget(moon, MoonTarget.CORAL_HIGH),
            ChuteCommands.retract(chute),
            MoonCommands.moveToTarget(moon, MoonTarget.CORAL_LOW))
        .onlyIf(() -> climber.getClimberCtrl().readyToClimb());
  }
}
