package frc.robot.subsystems.moon;

import static frc.robot.subsystems.moon.MoonConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MoonIOSim implements MoonIO {
    private final LinearSystem<N2, N1, N2> linearSystem;
    private final DCMotorSim motorSim;
    private double appliedVolts;

  public MoonIOSim() {
    linearSystem = LinearSystemId.createDCMotorSystem(kMotor, kJKgMetersSquared, kMotorReduction);
    motorSim = new DCMotorSim(linearSystem, kMotor);
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(MoonIOInputs inputs) {
    motorSim.update(kSimUpdateInterval);
    inputs.connected = true;
    inputs.positionRadians = motorSim.getAngularPositionRad();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.rpm = motorSim.getAngularVelocityRPM();
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();

    if (inputs.positionRadians <=0 ) {
            motorSim.setState(VecBuilder.fill(0.0, 0.0));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    motorSim.setInputVoltage(voltage);
  }
}
