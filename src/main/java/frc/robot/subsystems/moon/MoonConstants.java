package frc.robot.subsystems.moon;

import edu.wpi.first.math.system.plant.DCMotor;

public class MoonConstants {
    public static final double kMotorReduction = 1;
    public static final double kJKgMetersSquared = 0.025;
    public static final double kSimUpdateInterval = 0.02;
    public static final DCMotor kMotor = new DCMotor(12.0, 2.98, 150.0, 1.5, 710.0, 1);
}
