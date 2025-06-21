import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteCtrlSystem;
import frc.robot.subsystems.chute.ChuteIOSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ChuteTests {
  private Chute chute;
  private ChuteCtrlSystem chuteCtrl;
  private ChuteIOSim chuteSim;

  @BeforeEach
  void setup() {
    chuteCtrl = new ChuteCtrlSystem();
    chute = new Chute(chuteCtrl);
  }

  @Test
  void testExtend() {
    chuteCtrl.extend();
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      chute.periodic();
    }

    assertTrue(chuteCtrl.isExtended());
  }

  @Test
  void testRetract() {
    chuteCtrl.extend();
    double totalTime = 10.0; // seconds
    double dt = 0.02; // 20ms loop
    int iterations = (int) (totalTime / dt);

    for (int i = 0; i < iterations; i++) {
      chute.periodic();
    }

    if (!chuteCtrl.isExtended()) {
      fail("Chute didn't extend");
    }

    chuteCtrl.retract();
    for (int i = 0; i < iterations; i++) {
      chuteCtrl.periodic();
    }
    assertTrue(chuteCtrl.isRetracted());
  }
}
