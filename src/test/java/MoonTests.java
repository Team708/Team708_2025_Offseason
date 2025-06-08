import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonCtrl;
import frc.robot.subsystems.moon.MoonCtrlSystem;
import frc.robot.subsystems.moon.MoonIOSim;
import org.junit.jupiter.api.BeforeEach;

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
}
