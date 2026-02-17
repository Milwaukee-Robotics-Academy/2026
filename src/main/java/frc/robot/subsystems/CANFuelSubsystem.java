// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FuelConstants;

import static frc.robot.Constants.FuelConstants.*;

public class CANFuelSubsystem extends SubsystemBase {
  /** Left launcher motor controller (brushless) */
  private final SparkMax LeftIntakeLauncher;
  /** Right launcher motor controller (brushless) */
  private final SparkMax RightIntakeLauncher;
  /** Indexer / feeder motor controller (brushed) */
  private final SparkMax Indexer;

  /**
   * Construct the CANFuelSubsystem.
   *
   * This sets up motor controllers for the intake/launcher and indexer, applies
   * default SmartDashboard tuning values, and configures current limits and
   * motor inversion where appropriate.
   */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);
    RightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherConfig.inverted(true);
    LeftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
    // SmartDashboard.putNumber("Spin-up feeder roller value",
    // SPIN_UP_FEEDER_VOLTAGE);
  }

  /**
   * Set the output for both launcher/intake motors.
   *
   * @param power Motor output in the range supported by SparkMax (typically
   *              -1.0..1.0)
   */
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(power); // positive for shooting
  }

  /**
   * Set the output for the indexer / feeder roller.
   *
   * @param power Motor output in the range supported by SparkMax (typically
   *              -1.0..1.0)
   */
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  /**
   * Stop all rollers immediately.
   */
  public void stop() {
    Indexer.set(0);
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  /**
   * Returns a command that runs the intake sequence while held.
   *
   * Values are read from SmartDashboard keys so they can be tuned at runtime.
   *
   * @return a Command that starts the intake and stops it when
   *         finished/interrupted
   */
  public Command intakeCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double intakePercent = SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
          setIntakeLauncherRoller(intakePercent);
          setFeederRoller(feederPercent);
        },
        this::stop,
        this).withName("Intake");
  }

  /**
   * Returns a command that runs intake+feeder in reverse to eject balls.
   *
   * @return a Command that runs the eject routine until interrupted
   */
  public Command ejectCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double intakePercent = SmartDashboard.getNumber("Intaking intake roller value", INTAKE_INTAKING_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
          // reverse the intake and feeder to eject
          setIntakeLauncherRoller(-intakePercent);
          setFeederRoller(-feederPercent);
        },
        this::stop,
        this).withName("Eject");
  }

  /**
   * Returns a command to spin up the launcher wheels to a shooting setpoint.
   *
   * Launcher and feeder values are read from SmartDashboard.
   *
   * @return a Command that spins up the launcher until interrupted
   */
  public Command spinUpCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          double launcherPercent = SmartDashboard.getNumber("Launching launcher roller value",
              LAUNCHING_LAUNCHER_PERCENT);
          double feederPercent = SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
          setIntakeLauncherRoller(launcherPercent);
          setFeederRoller(feederPercent);
        },
        this::stop,
        this).withName("SpinUp");
  }

  /**
   * Returns a command that runs launcher+feeder for launching until interrupted.
   *
   * @return a Command that runs the full launch routine
   */
  public Command launchCommand() {
    return new edu.wpi.first.wpilibj2.command.StartEndCommand(
        () -> {
          setIntakeLauncherRoller(
              SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
          setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
        },
        this::stop,
        this).withName("Launch");

  }

  /**
   * Convenience sequence that spins up the launcher for a configured timeout,
   * then runs the full launch command.
   *
   * @return a SequentialCommandGroup performing spin-up then launch
   */
  public Command launchSequenceCommand() {
    // Spin up the launcher briefly, then run launcher+feeder to shoot until
    // interrupted
    return new edu.wpi.first.wpilibj2.command.SequentialCommandGroup(
        spinUpCommand().withTimeout(FuelConstants.SPIN_UP_SECONDS),
        launchCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
