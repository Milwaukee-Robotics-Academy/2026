package frc.robot.subsystems;

//import java.security.PrivateKey;
//import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase{
    // intake motor
    private SparkMax m_motor_9;
    // arm motor
    private SparkMax m_motor_10;
    
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
    

    // methods to control the intake wheels
    private void forwardIntake() {
        m_motor_9.set(0.5);
    }
    private void reverseIntake() {
        m_motor_9.set(-0.5);
    }
    private void stopIntake() {
        m_motor_9.set(0);
    }

    // methods to control the ar
    private void lowerArm() {
        m_motor_10.set(0.25);
    }

    private void raiseArm() {
        m_motor_10.set(-0.25);
    }

    private void stopArm() {
        m_motor_10.set(0);
    }


    public Command lowerArmCommand(){
    return new RunCommand(this::lowerArm, this).withName("LowerArm");
    }

    public Command raiseArmCommand(){
    return new RunCommand(this::raiseArm, this).withName("RaiseArm");
    }

    public Command stopArmCommand(){
    return new RunCommand(this::stopArm, this).withName("StopArm");
    }


    public Command forwardIntakeCommand(){
    return new RunCommand(this::forwardIntake, this).withName("ForwardIntake");
    }

    public Command reverseIntakeCommand(){
    return new RunCommand(this::reverseIntake, this).withName("ReverseIntake");
    }

    public Command stopIntakeCommand(){
    return new RunCommand(this::stopIntake, this).withName("StopIntake");
    }

    public Trigger forwardIntakeTrigger(){
    return new Trigger(() -> (intakeForwarding()));
    }

    public boolean intakeForwarding(){
        return m_motor_10.get() > 0.1;
    }

    public Trigger reverseIntakeTrigger(){
    return new Trigger(() -> (intakeReversing()));
    }

    public boolean intakeReversing(){
        return m_motor_10.get() < -0.1;
    }

     public Trigger stopIntakeTrigger(){
        return new Trigger(() -> (!intakeStopping()));
    }
    public boolean intakeStopping(){
        return m_motor_10.get() > 0;
    }
    //Arm Triggers
        public Trigger raiseArmTrigger(){
    return new Trigger(() -> (armRaising()));
    }

    public boolean armRaising(){
        return m_motor_9.get() > 0.1;
    }

    public Trigger lowerArmTrigger(){
    return new Trigger(() -> (armLowering()));
    }

    public boolean armLowering(){
        return m_motor_9.get() > -0.1;
    }

     public Trigger middleArmTrigger(){
        return new Trigger(() -> (!armMiddled()));
    }
    public boolean armMiddled(){
        return m_motor_9.get() > 0;
    }
}