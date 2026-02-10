package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter {
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
}