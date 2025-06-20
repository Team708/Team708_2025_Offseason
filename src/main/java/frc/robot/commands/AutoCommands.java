package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.feederLeftBlue;
import static frc.robot.subsystems.drive.DriveConstants.feederLeftRed;
import static frc.robot.subsystems.drive.DriveConstants.feederRightBlue;
import static frc.robot.subsystems.drive.DriveConstants.feederRightRed;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.moon.Moon;

public class AutoCommands {
  public static Command left3Coral(Drive drive, Elevator elevator, Moon moon, Intake intake) {
    Pose2d startingPose;
    Pose2d feederPose;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      startingPose = DriveConstants.startingPoseLeftRed;
      feederPose = feederLeftRed;

    } else {
      startingPose = DriveConstants.startingPoseLeftBlue;
      feederPose = feederLeftBlue;
    }
    return Commands.sequence(
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 10),
            DriveCommands.driveToPose(feederPose, drive),
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 11),
            DriveCommands.driveToPose(feederPose, drive),
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 12))
        .beforeStarting(() -> drive.setPose(startingPose));
  }

  public static Command right3Coral(Drive drive, Elevator elevator, Moon moon, Intake intake) {
    Pose2d startingPose;
    Pose2d feederPose;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      startingPose = DriveConstants.startingPoseRightRed;
      feederPose = feederRightRed;
    } else {
      startingPose = DriveConstants.startingPoseRightBlue;
      feederPose = feederRightBlue;
    }
    return Commands.sequence(
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 5),
            DriveCommands.driveToPose(feederPose, drive),
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 4),
            DriveCommands.driveToPose(feederPose, drive),
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 3))
        .beforeStarting(() -> drive.setPose(startingPose));
  }

  public static Command center1Coral(Drive drive, Elevator elevator, Moon moon, Intake intake) {
    Pose2d startingPose;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      startingPose = DriveConstants.startingPoseCenterRed;
    } else {
      startingPose = DriveConstants.startingPoseCenterBlue;
    }

    return Commands.sequence(
            IntakeCommands.intakeCoral(intake),
            CompositeCommands.scoreCoral(drive, elevator, moon, intake, 8))
        .beforeStarting(() -> drive.setPose(startingPose));
  }
}
