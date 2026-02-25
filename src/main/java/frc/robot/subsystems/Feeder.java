package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder extends SubsystemBase {
    
    private SparkMax m_motor_11; // feeder motor

    private static final double FEEDER_SPEED_FORWARD = -0.5;         // negative = forward
    private static final double FEEDER_SPEED_REVERSE = 0.5;          // positive = reverse


    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================

    public Feeder() {

        // initialize motor 11, 12, and 13 as a SparkMax motor
        m_motor_11 = new SparkMax(11, MotorType.kBrushless); // feeder motor

        // FEEDER MOTOR CONFIG (motor 11)
        SparkMaxConfig motor_11_config = new SparkMaxConfig();
        motor_11_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        m_motor_11.configure(motor_11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== FEEDER METHODS ====================

    // Set feeder motor speeds
    private void forwardFeeder() {
        m_motor_11.set(FEEDER_SPEED_FORWARD);
    }
    private void reverseFeeder() {
        m_motor_11.set(FEEDER_SPEED_REVERSE);
    }
    private void stopFeeder() {
        m_motor_11.set(0);
    }    
  
    // ==================== FEEDER MOTOR COMMANDS ====================

    public Command forwardFeederCommand() {
        return new RunCommand(this::forwardFeeder, this).withName("ForwardFeeder");
    }

    public Command reverseFeederCommand() {
        return new RunCommand(this::reverseFeeder, this).withName("ReverseFeeder");
    }

    public Command stopFeederCommand() {
        return new RunCommand(this::stopFeeder, this).withName("StopFeeder");
    }

    @Override
    public void periodic() {
       
    }

}