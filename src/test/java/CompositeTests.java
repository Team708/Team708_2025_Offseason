import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCtrlSystem;
import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonCtrlSystem;
import org.junit.jupiter.api.BeforeEach;

public class CompositeTests {
  private Elevator elevator;
  private Moon moon;

  @BeforeEach
  void setup() {
    elevator = new Elevator(new ElevatorCtrlSystem());
    moon = new Moon(new MoonCtrlSystem());
  }
}
