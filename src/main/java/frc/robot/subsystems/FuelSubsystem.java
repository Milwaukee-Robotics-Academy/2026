// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;
import static frc.robot.Constants.FuelConstants.*;



public class FuelSubsystem extends SubsystemBase {

  private SparkMax leftShooter;
  private SparkMaxConfig leftShooterConfig;
  private SparkClosedLoopController leftShooterController;

  private SparkMax rightShooter;
  private SparkMaxConfig rightShooterConfig;
  private SparkClosedLoopController rightShooterController;

  private SparkMax indexer;
  private SparkMaxConfig indexerConfig;
  private SparkClosedLoopController indexerController;
  /**
   * Construct the CANFuelSubsystem.
   *
   * This sets up motor controllers for the intake/launcher and indexer, applies
   * default SmartDashboard tuning values, and configures current limits and
   * motor inversion where appropriate.
   */
  public FuelSubsystem() {

    leftShooter = new SparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    leftShooterConfig = new SparkMaxConfig();
    leftShooterConfig
        .smartCurrentLimit(SHOOTER_MOTOR_CURRENT_LIMIT)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast);
    leftShooterController = leftShooter.getClosedLoopController();
    
    rightShooter = new SparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
    rightShooterConfig = new SparkMaxConfig();
    rightShooterController = rightShooter.getClosedLoopController();

    indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
    indexerConfig = new SparkMaxConfig();
    indexerController = indexer.getClosedLoopController();

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INDEXER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Intaking intake roller value", SHOOTER_INTAKING_PERCENT);
    SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT);
    SmartDashboard.putNumber("Launching launcher roller value", SHOOTER_LAUNCHING_PERCENT);
    // SmartDashboard.putNumber("Spin-up feeder roller value",
    // SPIN_UP_FEEDER_VOLTAGE);
  }

  private void intake() {
    indexer.set(Constants.FuelConstants.SHOOTER_INTAKING_PERCENT);
  }

  private void spinUp() {
  }

  private void launch() {
    indexer.set(Constants.FuelConstants.INDEXER_LAUNCHING_PERCENT);
  }

  private void eject() {
    indexer.set(-Constants.FuelConstants.INDEXER_INTAKING_PERCENT);
  }


  /**
   * Stop all rollers immediately.
   */
  public void stop() {
    indexer.set(0);
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
    return new RunCommand(
      this::intake,
        this).withName("Intake");
  }

  /**
   * Returns a command that runs intake+feeder in reverse to eject balls.
   *
   * @return a Command that runs the eject routine until interrupted
   */
  public Command ejectCommand() {
    return new RunCommand(this::eject,this).withName("Eject");
  }

  /**
   * Returns a command to spin up the launcher wheels to a shooting setpoint.
   *
   * Launcher and feeder values are read from SmartDashboard.
   *
   * @return a Command that spins up the launcher until interrupted
   */
  public Command spinUpCommand() {
    return new RunCommand(this::spinUp,this).withName("SpinUp");
  }

  /**
   * Returns a command that runs launcher+feeder for launching until interrupted.
   *
   * @return a Command that runs the full launch routine
   */
  // public Command launchCommand() {
  //   return new edu.wpi.first.wpilibj2.command.StartEndCommand(
  //       () -> {
  //         setIntakeLauncherRoller(
  //             SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT));
  //         setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", INDEXER_LAUNCHING_PERCENT));
  //       },
  //       this::stop,
  //       this).withName("Launch");

  // }

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
         new WaitCommand(SPIN_UP_SECONDS),
        launchCommand());
  }

  /**
   * Launch command: runs launcher + feeder until interrupted.
   * Extracted from previous commented code so launchSequenceCommand can call it.
   */
  public Command launchCommand() {
    return new RunCommand(this::launch,this).withName("Launch");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command stopCommand() {
    return new RunCommand(this::stop, this).withName("Stop");
  }
}
