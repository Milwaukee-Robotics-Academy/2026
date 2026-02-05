package frc.robot.subsystems;

import java.security.PrivateKey;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake {
    // create a new variable for motor 9
    private SparkMax m_motor_9;

    // this method controls the intake wheels
    public Intake() {
        //initialize motor 9 as a SparkMax motor
        m_motor_9 = new SparkMax(9, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_9_config = new SparkMaxConfig();

        //configure motor settings
        global_config
            //only use 40 if NOT 550 
            .smartCurrentLimit(40);
            //learn more about idle mode in the API docs
            .idleMode(IdleMode.kBrake);
            //learn more about persist mode in the API docs
            .persistMode(PersistMode.kPersist);

        motor_9_config
            .apply(global_config);

        m_motor_9.configure(motor_9_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }

    public Arm(){
         //initialize motor 10 as a SparkMax motor
        m_motor_10 = new SparkMax(10, MotorType.kBrushless);

        //set up configs for SparkMax motors
        SparkMaxConfig global_config = new SparkMaxConfig();
        SparkMaxConfig motor_9_config = new SparkMaxConfig();

        //configure motor settings
        global_config
            //only use 40 if NOT 550 
            .smartCurrentLimit(40);
            //learn more about idle mode in the API docs
            .idleMode(IdleMode.kBrake);
            //learn more about persist mode in the API docs
            .persistMode(PersistMode.kPersist);

        motor_10_config
            .apply(global_config);

        m_motor_10.configure(motor_10_config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);    
    }private void()

    {
        intakeState = intakeStates.UNLOCKED;
        encoderLockValue = 0;
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


    // methods to control the arm
    public void lowerArm() {
        m_motor_10.set(-0.25);
    }

    private void raiseArm() {
        m_motor_10.set(0.25);
    }

    public void stopArm() {
        m_motor_10.set(0);
    }
}
