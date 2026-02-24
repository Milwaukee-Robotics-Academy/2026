// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  //private final Vision m_vision;

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */

   //TODO: check withControllerRotationAxis needs to be inverted
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));
  
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(
        m_drivebase.getSwerveDrive(),
        () -> driverXbox.getLeftY(),       // Not inverted
        () -> driverXbox.getLeftX()        // Not inverted
    )
    .withControllerRotationAxis(() -> driverXbox.getRightX())  // Not inverted
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //m_vision = new Vision(m_drivebase.getSwerveDrive());
    
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", m_drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // commands created as local variables to setup drive orientation
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = m_drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    // commands to consider if we try a different mode for shooter
    // Command driveFieldOrientedDirectAngle      = m_drivebase.driveFieldOriented(driveDirectAngle);
    // Command driveRobotOrientedAngularVelocity  = m_drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    // Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    // Command driveSetpointGenKeyboard = m_drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

   // set default commands
    m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    //m_shooter.setDefaultCommand(m_shooter.stopShooterCommand());
    m_shooter.setDefaultCommand(m_shooter.stopAllCommand());
    m_intake.setDefaultCommand(m_intake.stopIntakeCommand());

    if (RobotBase.isSimulation()) {
    m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
        m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }


    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //m_drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> m_drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(m_drivebase.sysIdDriveMotorCommand());
      //driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
      //                                               () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
      

//      driverXbox.b().whileTrue(
//          m_drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      //driverXbox.y().whileTrue(m_drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      driverXbox.back().whileTrue(m_drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

      // test to determine shooter directions; motor 13 should automatically follow
      //operatorXbox.rightBumper().whileTrue(m_shooter.forwardShooterCommand());

    } else
    {
      // ==================== DRIVER COMMANDS ====================

      driverXbox.a().onTrue((Commands.runOnce(m_drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(m_drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(m_drivebase::lock, m_drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      //driverXbox.y().onTrue(m_drivebase.driveToDistanceCommandDefer(m_drivebase::getPose, 2, 14));
      driverXbox.y().whileTrue(m_drivebase.driveForward());

      // ==================== OPERATOR COMMANDS ====================
      //intake wheels 
      operatorXbox.y().whileTrue(m_intake.forwardIntakeCommand());
      operatorXbox.a().whileTrue(m_intake.reverseIntakeCommand()); 

      //arm with encoder setpoints
      // operatorXbox.povDown().onTrue(m_intake.armDownCommand());
      // operatorXbox.povUp().onTrue(m_intake.armUpCommand());
      // operatorXbox.povRight().onTrue(m_intake.armMiddleCommand()); 

      //arm with manual control (testing/override, no safety)
      operatorXbox.povDown().onTrue(m_intake.armSpeedDownCommand());
      operatorXbox.povUp().onTrue(m_intake.armSpeedUpCommand());

      //shooter
      // Option 1: Right Trigger - Smart feeder (auto-pauses and resumes)
      //operatorXbox.rightTrigger().whileTrue(m_shooter.smartFeederCommand());

      // Option 2: Right Bumper - Spin up shooter (tesing)
      operatorXbox.rightBumper().whileTrue(m_shooter.forwardAllCommand());

      // Option 3: Left Bumper - Force feeder (testing/override, no safety)
      //operatorXbox.leftBumper().whileTrue(m_shooter.forwardFeederCommand());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  private void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }
}
