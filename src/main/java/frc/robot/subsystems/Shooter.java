package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
    // feeder motor
    //private SparkMax m_motor_11;

    //shooter motors
    private SparkMax m_motor_12;
    private SparkMax m_motor_13;
    private SparkMax m_motor_11;



    // this method controls the intake wheels
    public Shooter() {
        //initialize motor 12 and 13 as a SparkMax motor
        m_motor_12 = new SparkMax(12, MotorType.kBrushless);
        m_motor_13 = new SparkMax(13, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_12_config = new SparkMaxConfig();
        SparkMaxConfig motor_13_config = new SparkMaxConfig();

        //configure motor settings
        global_config
            .smartCurrentLimit(40) // only use 40 if NOT 550 motor
            .idleMode(IdleMode.kBrake);

        motor_12_config
            .apply(global_config);
        motor_13_config
            .apply(global_config);

        m_motor_12.configure(motor_12_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_13.configure(motor_13_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

    public void Feeder() {
         //initialize motor 11 as a SparkMax motor
        m_motor_11 = new SparkMax(11, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_11_config = new SparkMaxConfig();

        //configure motor settings
         global_config
            .smartCurrentLimit(40) // only use 40 if NOT 550 motor
            .idleMode(IdleMode.kBrake);

        motor_11_config
            .apply(global_config);

        m_motor_11.configure(motor_11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
        }

    private void forwardShooter() {
        m_motor_12.set(0.5);
        m_motor_13.set(0.5);
    }
    private void reverseShooter() {
        m_motor_12.set(-0.5);
        m_motor_13.set(-0.5);
    }
    private void stopShooter() {
        m_motor_12.set(0);
        m_motor_13.set(0);
    }

    private void forwardFeeder() {
        m_motor_11.set(0.5);
    }
    private void reverseFeeder() {
        m_motor_11.set(-0.5);
    }
    private void stopFeeder() {
        m_motor_11.set(0);
    }

    public Command forwardFeederCommand(){
    return new RunCommand(this::forwardFeeder, this).withName("ForwardFeeder");
    }

    public Command reverseFeederCommand(){
    return new RunCommand(this::reverseFeeder, this).withName("ReverseFeeder");
    }

    public Command stopFeederCommand(){
    return new RunCommand(this::stopFeeder, this).withName("StopFeeder");
    }


    public Command forwardShooterCommand(){
    return new InstantCommand(this::forwardShooter, this).withName("ForwardShooter");
    }

    public Command reverseShooterCommand(){
    return new InstantCommand(this::reverseShooter, this).withName("ReverseShooter");
    }

    public Command stopShooterCommand(){
    return new InstantCommand(this::stopShooter, this).withName("StopShooter");
    }
    


    public Trigger forwardIFeederTrigger(){
    return new Trigger(() -> (feederForwarding()));
    }

    public boolean feederForwarding(){
        return m_motor_11.get() > 0.1;
    }

    public Trigger reverseFeederTrigger(){
    return new Trigger(() -> (feederReversing()));
    }

    public boolean feederReversing(){
        return m_motor_11.get() > -0.1;
    }

     public Trigger stopFeederTrigger(){
        return new Trigger(() -> (!feederStopping()));
    }
    public boolean feederStopping(){
        return m_motor_11.get() == 0;
    }
    //Feeder Triggers
        public Trigger forwardShooterTrigger(){
    return new Trigger(() -> (shooterForwarding()));
    }

    public boolean shooterForwarding(){
       return m_motor_12.get() > 0.1 && m_motor_13.get() > 0.1;
    }

    public Trigger shooterReversingTrigger(){
    return new Trigger(() -> (shooterReversing()));
    }

    public boolean shooterReversing(){
        return m_motor_12.get() < -0.1 && m_motor_13.get() < -0.1;
    }

     public Trigger stopShooterTrigger(){
        return new Trigger(() -> (!shooterStopping()));
    }
    public boolean shooterStopping(){
        return m_motor_12.get() == 0 && m_motor_13.get() == 0;
    }
}