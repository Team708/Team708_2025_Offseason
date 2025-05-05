package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.*;

public class ScoreSequence extends SequentialCommandGroup {
  public ScoreSequence(Pose2d pose, Drive drive) {
    addCommands(DriveCommands.driveToPose(pose, drive));
  }
}
