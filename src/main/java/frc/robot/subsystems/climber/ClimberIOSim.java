package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableBoolean;

public class ClimberIOSim implements ClimberIO {
  LoggedTunableBoolean cageLimit1 = new LoggedTunableBoolean("Climber/cageLimit1", true);
  LoggedTunableBoolean cageLimit2 = new LoggedTunableBoolean("Climber/cageLimit2", true);
  LoggedTunableBoolean beamBreak1 = new LoggedTunableBoolean("Climber/beamBreak1", true);
  LoggedTunableBoolean beamBreak2 = new LoggedTunableBoolean("Climber/beamBreak2", true);
  private final LinearSystem<N2, N1, N2> linearSystem;
  private final DCMotorSim climberSim;
  private double appliedVolts;
  private boolean servoUnlocked = true;

  public ClimberIOSim() {
    linearSystem =
        LinearSystemId.createDCMotorSystem(
            Constants.K1_VORTEX, J_KG_METERS_SQUARED, MOTOR_REDUCTION);
    climberSim = new DCMotorSim(linearSystem, Constants.K1_VORTEX);
    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.update(SIM_UPDATE_INTERVAL);
    inputs.connected = true;
    inputs.positionRadians = climberSim.getAngularPositionRad();
    inputs.positionDegrees = Math.toDegrees(inputs.positionRadians);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = climberSim.getCurrentDrawAmps();
    inputs.rpm = climberSim.getAngularVelocityRPM();
    inputs.servoUnlocked = servoUnlocked;
    inputs.cageLimit1 = cageLimit1.get();
    inputs.cageLimit2 = cageLimit2.get();
    inputs.beamBreak1 = beamBreak1.get();
    inputs.beamBreak2 = beamBreak2.get();

    if (inputs.positionRadians <= 0.001 && inputs.appliedVolts < 0) {
      climberSim.setState(VecBuilder.fill(0.0, 0.0));
    }

    if (inputs.positionRadians <= 0) {
      inputs.reverseLimitReached = true;
    } else {
      inputs.reverseLimitReached = false;
    }

    if (inputs.positionRadians >= (EXTENDED_RADIANS - 0.1)) {
      inputs.forwardLimitReached = true;
    } else {
      inputs.forwardLimitReached = false;
    }
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    climberSim.setInputVoltage(voltage);
  }

  @Override
  public void setServo(boolean isUnlocked) {
    servoUnlocked = isUnlocked;
  }
}
