package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.UnitUtil;

public class ElevatorIOSim implements ElevatorIO {
  private PIDController controller;
  private ElevatorSim elevatorSim;

  public ElevatorIOSim() {
    elevatorSim =
        new ElevatorSim(
            Constants.K2_VORTEX,
            MOTOR_REDUCTION,
            UnitUtil.poundsToKilograms(CARRAIGE_MASS_LBS),
            UnitUtil.inchesToMeters(EFFECTIVE_DRUM_RADIUS_INCHES),
            UnitUtil.inchesToMeters(MIN_HEIGHT_INCHES),
            UnitUtil.inchesToMeters(MAX_HEIGHT_INCHES),
            SIMULATE_GRAVITY,
            UnitUtil.inchesToMeters(STARTING_HEIGHT_INCHES));
    controller = new PIDController(KP, KI, KD);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(SIM_UPDATE_INTERVAL);
    double setPoint = controller.calculate(inputs.positionInches, inputs.targetInches);
    elevatorSim.setInputVoltage(MathUtil.clamp(setPoint, -MAX_VOLTAGE_SIM, MAX_VOLTAGE_SIM));
    inputs.motor1Connected = true;
    inputs.motor2Connected = true;
    inputs.appliedVolts = MathUtil.clamp(setPoint, -MAX_VOLTAGE_SIM, MAX_VOLTAGE_SIM);
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionInches = UnitUtil.metersToInches(elevatorSim.getPositionMeters());
    inputs.velocityInchesPerSecond =
        UnitUtil.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    inputs.rpm =
        (UnitUtil.metersToInches(elevatorSim.getVelocityMetersPerSecond()) * 60 * MOTOR_REDUCTION)
            / (2 * Math.PI * EFFECTIVE_DRUM_RADIUS_INCHES);

    // Hard limits
    if (inputs.positionInches <= 0 && inputs.velocityInchesPerSecond < 0) {
      elevatorSim.setState(VecBuilder.fill(0.0, 0.0));
    } else if (inputs.positionInches >= MAX_HEIGHT_INCHES && inputs.velocityInchesPerSecond > 0) {
      elevatorSim.setState(VecBuilder.fill(MAX_HEIGHT_INCHES, 0.0));
    }

    // Zero triggered
    if (inputs.positionInches <= 0) {
      inputs.reverseLimitTriggered = true;
    } else {
      inputs.reverseLimitTriggered = false;
    }
  }
}
