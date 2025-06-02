package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorTarget;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;

public class CompositeCommands {
  public static Command score(
      Drive drive, Elevator elevator, Moon moon, Intake intake, int LocationID) {
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
            ElevatorCommands.moveToLevel(elevator, ElevatorTarget.CORAL_L4),
            MoonCommands.moveToPosition(moon, MoonTarget.CORAL_HIGH)),
        IntakeCommands.outakeCoral(intake),
        MoonCommands.moveToPosition(moon, MoonTarget.CORAL_LOW),
        ElevatorCommands.moveToLevel(elevator, ElevatorTarget.CORAL_L0));
  }
}
