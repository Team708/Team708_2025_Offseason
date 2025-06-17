package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableBoolean;

public class IntakeIOSim implements IntakeIO {
  private final LoggedTunableBoolean beamTriggered;
  private final LinearSystem<N2, N1, N2> linearSystem;
  private final DCMotorSim intakeSim;
  private double appliedVolts;

  public IntakeIOSim() {
    linearSystem =
        LinearSystemId.createDCMotorSystem(Constants.k1Vortex, kJKgMetersSquared, kMotorReduction);
    intakeSim = new DCMotorSim(linearSystem, Constants.k1Vortex);
    appliedVolts = 0.0;
    beamTriggered = new LoggedTunableBoolean("Intake/BeamTriggered", false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(kSimUpdateInterval);
    intakeSim.setInputVoltage(appliedVolts);
    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = intakeSim.getCurrentDrawAmps();
    inputs.rpm = intakeSim.getAngularVelocityRPM();
    inputs.positionRad = intakeSim.getAngularPositionRad();
    inputs.beamTriggered = beamTriggered.get();
  }

  public void setVoltage(double voltage) {
    appliedVolts = voltage;
  }
}
