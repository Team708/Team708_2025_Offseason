import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;
import frc.robot.subsystems.moon.MoonCtrl;
import frc.robot.subsystems.moon.MoonCtrlSystem;
import frc.robot.subsystems.moon.MoonIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MoonTests {
  private Moon moon;
  private MoonCtrl moonCtrl;
  private MoonIOSim moonSim;

  @BeforeEach
  void setup() {
    moonSim = new MoonIOSim();
    moonCtrl = new MoonCtrlSystem(moonSim);
    moon = new Moon(moonCtrl);
  }

  @Test
  void testMoveToPosition() {
    moonCtrl.setTargetPosition(MoonTarget.CORAL_HIGH);
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      moonCtrl.periodic();
    }

    assertTrue(moonCtrl.atTargetPosition());
  }
}
