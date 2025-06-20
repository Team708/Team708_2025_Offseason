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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.MoonCommands;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteCtrl;
import frc.robot.subsystems.chute.ChuteCtrlManual;
import frc.robot.subsystems.chute.ChuteCtrlSystem;
import frc.robot.subsystems.chute.ChuteIOReal;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberCtrl;
import frc.robot.subsystems.climber.ClimberCtrlManual;
import frc.robot.subsystems.climber.ClimberCtrlSystem;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCtrlSystem;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.moon.Moon;
import frc.robot.subsystems.moon.MoonCtrl;
import frc.robot.subsystems.moon.MoonCtrlManual;
import frc.robot.subsystems.moon.MoonCtrlSystem;
import frc.robot.subsystems.moon.MoonIOReal;
import frc.robot.subsystems.moon.MoonIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Moon moon;
  private final Intake intake;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final Joystick reefController = new Joystick(1);

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
                ? new Chute(new ChuteCtrlManual(new ChuteIOReal()))
                : new Chute(new ChuteCtrlSystem(new ChuteIOReal()));
        elevator =
            Constants.elevatorManualMode
                ? new Elevator(new ElevatorCtrlManual(new ElevatorIOReal()))
                : new Elevator(new ElevatorCtrlSystem(new ElevatorIOReal()));
        climber =
            Constants.climberManualMode
                ? new Climber(new ClimberCtrlManual(new ClimberIOReal()))
                : new Climber(new ClimberCtrlSystem(new ClimberIOReal()));
        moon =
            Constants.moonManualMode
                ? new Moon(new MoonCtrlManual(new MoonIOReal()))
                : new Moon(new MoonCtrlSystem(new MoonIOReal()));
        intake = new Intake(new IntakeCtrlSystem(new IntakeIOReal()));

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
        climber =
            Constants.climberManualMode
                ? new Climber(new ClimberCtrlManual(new ClimberIOSim()))
                : new Climber(new ClimberCtrlSystem(new ClimberIOSim()));
        moon =
            Constants.moonManualMode
                ? new Moon(new MoonCtrlManual(new MoonIOSim()))
                : new Moon(new MoonCtrlSystem(new MoonIOSim()));
        intake = new Intake(new IntakeCtrlSystem(new IntakeIOSim()));
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
        climber =
            new Climber(
                new ClimberCtrl() {
                  public void periodic() {}
                });
        moon =
            new Moon(
                new MoonCtrl() {
                  public void periodic() {}
                });
        intake = new Intake(new IntakeCtrlSystem(new IntakeIOSim()));
        break;
    }

    // fSet up auto routines
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
    autoChooser.addOption("Left 3 Coral", AutoCommands.left3Coral(drive, elevator, moon, intake));
    autoChooser.addOption("Right 3 Coral", AutoCommands.right3Coral(drive, elevator, moon, intake));
    autoChooser.addOption(
        "Center 1 Coral", AutoCommands.center1Coral(drive, elevator, moon, intake));

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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.a().onTrue(CompositeCommands.intakePiece(intake, moon));
    driverController.b().onTrue(CompositeCommands.outtakePiece(intake, moon));
    driverController.x().onTrue(CompositeCommands.changeMode(elevator, moon, intake));
    //  driverController.y().onTrue(ElevatorCommands.moveToTarget(elevator,
    // ElevatorTarget.CORAL_L0));

    // elevator.setDefaultCommand(-
    //     ElevatorCommands.manualControl(elevator, () -> -driverController.getRightY()));
    // climber.setDefaultCommand(
    //     ClimberCommands.manualControl(climber, () -> -driverController.getRightY()));
    // chute.setDefaultCommand(ChuteCommands.manualControl(chute, () ->
    moon.setDefaultCommand(MoonCommands.manualControl(moon, () -> -driverController.getRightY()));
    // driverController.getRightY()));

    // driverController.b().onTrue(IntakeCommands.intakeCoral(intake));

    // new JoystickButton(operatorController,
    // Button.kY.value).onTrue(CompositeCommands.moveToLevel(elevator, moon, ElevatorLevel.L0));
    // new JoystickButton(operatorController,
    // Button.kRightBumper.value).onTrue(CompositeCommands.moveToLevel(elevator, moon,
    // ElevatorLevel.L1));
    // new JoystickButton(operatorController,
    // Button.kX.value).onTrue(CompositeCommands.moveToLevel(elevator, moon, ElevatorLevel.L2));
    // new JoystickButton(operatorController,
    // Button.kLeftBumper.value).onTrue(CompositeCommands.moveToLevel(elevator, moon,
    // ElevatorLevel.L3));
    // new JoystickButton(operatorController,
    // Button.kRightStick.value).onTrue(CompositeCommands.moveToLevel(elevator, moon,
    // ElevatorLevel.L4));

    // new JoystickButton(reefController, 1)
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               DriveCommands.driveToPose(DriveConstants.poseMapBlue.get(1), drive).schedule();
    //             }));

    for (int i = 1; i < 13; i++) {
      new JoystickButton(reefController, i)
          .onTrue(
              DriveCommands.driveToPose(DriveConstants.poseMapBlue.get(i), drive)
                  .until(
                      () ->
                          Math.abs(driverController.getLeftY()) > 0.1
                              || Math.abs(driverController.getLeftX()) > 0.1
                              || Math.abs(driverController.getRightX()) > 0.1));
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
