// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ChuteCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteCtrl;
import frc.robot.subsystems.chute.ChuteCtrlManual;
import frc.robot.subsystems.chute.ChuteCtrlSystem;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.chute.ChuteIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCtrl;
import frc.robot.subsystems.elevator.ElevatorCtrlManual;
import frc.robot.subsystems.elevator.ElevatorCtrlSystem;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Chute chute;
  private final Elevator elevator;
  private final Climber climber;
  private final Manipulator manipulator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick reefController = new Joystick(1);

  // Pose maps
  HashMap<Integer, Pose2d> poseMapBlue = new HashMap<Integer, Pose2d>();
  HashMap<Integer, Pose2d> poseMapRed = new HashMap<Integer, Pose2d>();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        chute =
            Constants.chuteManualMode
                ? new Chute(new ChuteCtrlManual(new ChuteIOSpark()))
                : new Chute(new ChuteCtrlSystem(new ChuteIOSpark()));
        elevator =
            Constants.elevatorManualMode
                ? new Elevator(new ElevatorCtrlManual(new ElevatorIOReal()))
                : new Elevator(new ElevatorCtrlSystem(new ElevatorIOReal()));
        climber = new Climber(new ClimberIOSpark());
        manipulator = new Manipulator(new ManipulatorIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        chute =
            Constants.chuteManualMode
                ? new Chute(new ChuteCtrlManual(new ChuteIOSim()))
                : new Chute(new ChuteCtrlSystem(new ChuteIOSim()));
        elevator =
            Constants.elevatorManualMode
                ? new Elevator(new ElevatorCtrlManual(new ElevatorIOSim()))
                : new Elevator(new ElevatorCtrlSystem(new ElevatorIOSim()));
        climber = new Climber(new ClimberIOSim());
        manipulator = new Manipulator(new ManipulatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        chute =
            new Chute(
                new ChuteCtrl() {
                  public void periodic() {}
                });
        elevator =
            new Elevator(
                new ElevatorCtrl() {
                  public void periodic() {}
                });
        climber = new Climber(new ClimberIO() {});
        manipulator = new Manipulator(new ManipulatorIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    poseMapBlue.put(1, new Pose2d(3.12, 4.18, new Rotation2d(0)));
    poseMapBlue.put(2, new Pose2d(3.12, 3.84, new Rotation2d(0)));
    poseMapBlue.put(3, new Pose2d(3.63, 2.92, new Rotation2d(Math.PI / 3)));
    poseMapBlue.put(4, new Pose2d(3.96, 2.73, new Rotation2d(Math.PI / 3)));
    poseMapBlue.put(5, new Pose2d(5.04, 2.76, new Rotation2d((2 * Math.PI) / 3)));
    poseMapBlue.put(6, new Pose2d(5.32, 2.90, new Rotation2d((2 * Math.PI) / 3)));
    poseMapBlue.put(7, new Pose2d(5.88, 3.84, new Rotation2d(-Math.PI)));
    poseMapBlue.put(8, new Pose2d(5.88, 4.18, new Rotation2d(-Math.PI)));
    poseMapBlue.put(9, new Pose2d(5.32, 5.198, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapBlue.put(10, new Pose2d(5.04, 5.407, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapBlue.put(11, new Pose2d(3.96, 5.407, new Rotation2d(-Math.PI / 3)));
    poseMapBlue.put(12, new Pose2d(3.63, 5.198, new Rotation2d(-Math.PI / 3)));

    poseMapRed.put(1, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI)));
    poseMapRed.put(2, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI)));
    poseMapRed.put(3, new Pose2d(0.0, 0.0, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(4, new Pose2d(0.0, 0.0, new Rotation2d(-(2 * Math.PI) / 3)));
    poseMapRed.put(5, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(6, new Pose2d(0.0, 0.0, new Rotation2d(-Math.PI / 3)));
    poseMapRed.put(7, new Pose2d(0.0, 0.0, new Rotation2d(0)));
    poseMapRed.put(8, new Pose2d(0.0, 0.0, new Rotation2d(0)));
    poseMapRed.put(9, new Pose2d(0.0, 0.0, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(10, new Pose2d(0.0, 0.0, new Rotation2d(Math.PI / 3)));
    poseMapRed.put(11, new Pose2d(0.0, 0.0, new Rotation2d((2 * Math.PI) / 3)));
    poseMapRed.put(12, new Pose2d(0.0, 0.0, new Rotation2d((2 * Math.PI) / 3)));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.y().onTrue(ChuteCommands.extend(chute));
    controller.x().onTrue(ChuteCommands.retract(chute));
    controller.b().whileTrue(ChuteCommands.manualControl(chute, () -> controller.getRightY()));

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (alliance == Alliance.Blue) {
      for (int i = 1; i < 13; i++) {
        new JoystickButton(reefController, i)
            .onTrue(DriveCommands.driveToPose(poseMapBlue.get(i), drive));
      }
    } else {
      for (int i = 1; i < 13; i++) {
        new JoystickButton(reefController, i)
            .onTrue(DriveCommands.driveToPose(poseMapRed.get(i), drive));
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
