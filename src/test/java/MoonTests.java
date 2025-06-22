import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonConstants.MoonTarget;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class MoonTests {
  private Moon moon;

  @BeforeEach
  void setup() {
    moon = new Moon();
  }

  @Test
  void testMoveToPosition() {
    moon.getMoonCtrl().setTargetPosition(MoonTarget.CORAL_HIGH);
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      moon.periodic();
    }

    assertTrue(moon.getMoonCtrl().atTargetPosition());
  }
}
