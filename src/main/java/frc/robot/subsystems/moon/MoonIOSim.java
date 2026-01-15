package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class MoonIOSim implements MoonIO {
  private final LinearSystem<N2, N1, N2> linearSystem;
  private final DCMotorSim moonSim;
  private final PIDController controller;

  public MoonIOSim() {
    linearSystem =
        LinearSystemId.createDCMotorSystem(
            Constants.K1_VORTEX, J_KG_METERS_SQUARED, MOTOR_REDUCTION);
    moonSim = new DCMotorSim(linearSystem, Constants.K1_VORTEX);
    controller = new PIDController(KP, KI, KD);
  }

  @Override
  public void updateInputs(MoonIOInputs inputs) {
    double setPoint = controller.calculate(inputs.positionRadians, inputs.targetRadians);
    moonSim.setInputVoltage(MathUtil.clamp(setPoint, -MAX_VOLTAGE, MAX_VOLTAGE));
    moonSim.update(SIM_UPDATE_INTERVAL);
    inputs.connected = true;
    inputs.positionRadians = moonSim.getAngularPositionRad();
    inputs.positionDegrees = Math.toDegrees(inputs.positionRadians);
    inputs.appliedVolts = moonSim.getInputVoltage();
    inputs.currentAmps = moonSim.getCurrentDrawAmps();
    inputs.rpm = moonSim.getAngularVelocityRPM();

    if (inputs.positionRadians <= 0.001 && inputs.appliedVolts < 0) {
      moonSim.setState(VecBuilder.fill(0.0, 0.0));
    }

    if (inputs.positionRadians == 0) {
      inputs.reverseLimitReached = true;
    }

    if (inputs.positionRadians >= MAX_RADIANS) {
      inputs.forwardLimitReached = true;
    } else {
      inputs.forwardLimitReached = false;
    }
  }
}
