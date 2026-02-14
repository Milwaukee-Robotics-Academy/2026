package frc.robot.subsystems;

//import java.security.PrivateKey;
//import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    
    
    private SparkMax m_motor_9;  // intake motor
    private SparkMax m_motor_10; // arm motor

    // ==================== CONFIGURE INTAKE WHEEL MOTORS ====================
    
    public Intake() {
        //initialize motor 9 as a SparkMax motor
        m_motor_9 = new SparkMax(9, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_9_config = new SparkMaxConfig();

        //configure motor settings
        global_config
            .smartCurrentLimit(40) // only use 40 if NOT 550 motor
            .idleMode(IdleMode.kBrake);

        motor_9_config
            .apply(global_config);

        m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    // ==================== CONFIGURE ARM MOTORS ====================

    public void Arm(){
         //initialize motor 10 as a SparkMax motor
        m_motor_10 = new SparkMax(10, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_10_config = new SparkMaxConfig();

        //configure motor settings
         global_config
            .smartCurrentLimit(40) // only use 40 if NOT 550 motor
            .idleMode(IdleMode.kBrake);

        motor_10_config
            .apply(global_config);

        m_motor_10.configure(motor_10_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
    }
    
    // ==================== SET INTAKE WHEEL SPEED ====================

    private void forwardIntake() {
        m_motor_9.set(0.5);
    }
    private void reverseIntake() {
        m_motor_9.set(-0.5);
    }
    private void stopIntake() {
        m_motor_9.set(0);
    }

    // ==================== SET ARM MOTOR SPEED ====================

    private void lowerArm() {
        m_motor_10.set(0.25);
    }
    private void raiseArm() {
        m_motor_10.set(-0.25);
    }
    private void stopArm() {
        m_motor_10.set(0);
    }

    // ==================== INTAKE WHEEL COMMANDS ====================

    public Command forwardIntakeCommand(){
        return new RunCommand(this::forwardIntake, this).withName("ForwardIntake");
    }
    public Command reverseIntakeCommand(){
        return new RunCommand(this::reverseIntake, this).withName("ReverseIntake");
    }
    public Command stopIntakeCommand(){
        return new RunCommand(this::stopIntake, this).withName("StopIntake");
    }

    // ==================== ARM COMMANDS ====================

    public Command lowerArmCommand(){
        return new RunCommand(this::lowerArm, this).withName("LowerArm");
    }
    public Command raiseArmCommand(){
        return new RunCommand(this::raiseArm, this).withName("RaiseArm");
    }
    public Command stopArmCommand(){
        return new RunCommand(this::stopArm, this).withName("StopArm");
    }

}