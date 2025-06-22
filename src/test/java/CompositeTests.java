import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.moon.Moon;
import org.junit.jupiter.api.BeforeEach;

public class CompositeTests {
  private Elevator elevator;
  private Moon moon;

  @BeforeEach
  void setup() {
    elevator = new Elevator();
    moon = new Moon();
  }
}
