// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.INDEXER_INTAKING_PERCENT;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    SparkFlex intake = new SparkFlex(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

  /** Creates a new Intake. */
  public Intake() {
        intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(0.5)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(80);
    intakeConfig.closedLoop
         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
         // Set PID values for position control
         .p(SmartDashboard.getNumber("Shooter/kP", 0.0002))
         .outputRange(-1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private void intake() {
    intake.set(IntakeConstants.INTAKING_PERCENT);
  }

  private void eject() {
    intake.set(IntakeConstants.EJECT_PERCENT);
  }
}

