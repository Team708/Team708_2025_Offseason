import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCtrl;
import frc.robot.subsystems.elevator.ElevatorCtrlSystem;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import org.junit.jupiter.api.BeforeEach;

public class ElevatorTests {
  private Elevator elevator;
  private ElevatorCtrl elevatorCtrl;
  private ElevatorIOSim elevatorSim;

  @BeforeEach
  void setup() {
    elevatorSim = new ElevatorIOSim();
    elevatorCtrl = new ElevatorCtrlSystem(elevatorSim);
    elevator = new Elevator(elevatorCtrl);
  }
}
