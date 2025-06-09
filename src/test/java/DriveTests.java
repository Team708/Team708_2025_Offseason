import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DriveTests {
  private Drive drive;

  @BeforeEach
  void setup() {
    drive =
        new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
  }

  @Test
  void testDriveToPose() {
    Pose2d destPose = new Pose2d(5, 5, new Rotation2d());
    Command c =
        AutoBuilder.pathfindToPose(destPose, new PathConstraints(2.0, 2.0, Math.PI, Math.PI), 0.0);
    c.addRequirements(drive);
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);
    c.initialize();

    for (int i = 0; i < iterations; i++) {
      drive.periodic();
      c.execute();
      SimHooks.stepTiming(0.02);
    }

    assertTrue(drive.getPose().equals(destPose));
  }
}
