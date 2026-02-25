// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  //private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive).withName("Robot Drive Default"));

    /*configureButtonBindings();
    m_robotDrive.resetOdometry(new Pose2d()); // from old system

    // Configure default commands
    // Throttle should be connected to the joystick slider - higher value more speed. It should affect all axis
    m_robotDrive.setDefaultCommand(
            new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getY(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getX(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getZ(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    true,
                    false),
            m_robotDrive));*/

    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_shooter);

    SmartDashboard.putNumber("Bat Voltage", RobotController.getBatteryVoltage());

    SmartDashboard.putData("Intake", m_intake.runIntakeCommand().withName("Intake - Intaking"));
    SmartDashboard.putData("Extake", m_intake.runExtakeCommand().withName("Intake - Extaking"));

    SmartDashboard.putData("Feeder", m_shooter.runFeederCommand().withName("Shooter - Feeding and Shooting"));
    SmartDashboard.putData("Flywheel", m_shooter.runFlywheelCommand().withName("Shooter - Spinning up Flywheel"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // Right Trigger -> Run fuel intake in reverse
    m_driverController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Run fuel intake in reverse
    m_driverController
      .leftTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runExtakeCommand());

    // Y Button -> Run intake and run the shooter flywheel and feeder
    m_driverController.y().toggleOnTrue(m_shooter.runShooterCommand().alongWith(m_intake.runIntakeCommand()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  /*private void configureButtonBindings() {
    // Below are the various buttons for the joystick, change the button number to adjust which button it is connected to
    // Set swerve to X formation
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Run tube intake
    new JoystickButton(m_driverController, 1) // button 1 should be trigger
        .whileTrue(new RunCommand(() -> m_coralSubSystem.runIntakeCommand(), m_robotDrive));

    // Run tube intake in reverse
    new JoystickButton(m_driverController, 2) // button 2 should be button on top back of joystick
        .whileTrue(new RunCommand(() -> m_coralSubSystem.reverseIntakeCommand(), m_robotDrive));

    // Elevator/Arm to human player position, set ball intake to stow when idle
    new JoystickButton(m_driverController, 11) // 11 should be a button on the left side of the joystick, the top left button
        .whileTrue(new RunCommand(() -> m_coralSubSystem
        .setSetpointCommand(Setpoint.kFeederStation)
        .alongWith(m_algaeSubsystem.stowCommand()), m_robotDrive));

    // Elevator/Arm to level 2 position
    new JoystickButton(m_driverController, 14) // 14, 15, 16 should hopefully be the bottom row of buttons on the right side
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2), m_robotDrive));

    // Elevator/Arm to level 3 position
    new JoystickButton(m_driverController, 15)
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3), m_robotDrive));

    // Elevator/Arm to level 4 position
    new JoystickButton(m_driverController, 16)
        .whileTrue(new RunCommand(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4), m_robotDrive));

    // Run ball intake, set to leave out when idle
    new JoystickButton(m_driverController, 3) // button 3 should be top left on joystick
        .whileTrue(new RunCommand(() -> m_algaeSubsystem.runIntakeCommand(), m_robotDrive));

    // Run ball intake in reverse, set to stow when idle
    new JoystickButton(m_driverController, 4) // 4 should be top right on joystick
        .whileTrue(new RunCommand(() -> m_algaeSubsystem.reverseIntakeCommand(), m_robotDrive));

    // Zero swerve heading
    new JoystickButton(m_driverController, 7) // 7 should be on the left side of the joystick
    .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeadingCommand(), m_robotDrive));   
  }*/

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_robotDrive);
  }
}
