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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    
    private SparkMax m_motor_11; // feeder motor
    private SparkMax m_motor_12; // shooter motor 1 (leader)
    private SparkMax m_motor_13; // shooter motor 2 (follower)

    private RelativeEncoder m_encoder_12; // encoder motor 1 (only for leader motor, since follower will mirror it)

    // Target shooter speed and tolerance (adjust as needed based on testing)
    private static final double TARGET_SHOOTER_RPM = 4000.0;  // Adjust this!
    private static final double SPEED_TOLERANCE_RPM = 100.0;  // Within 100 RPM = ready 

    private static final double SHOOTER_SPEED_FORWARD = -0.75;       // forward is negative
    private static final double SHOOTER_SPEED_REVERSE = 0.5;         // reverse is positive

    private static final double FEEDER_SPEED_FORWARD = -0.5;         // forward is negative
    private static final double FEEDER_SPEED_REVERSE = 0.5;          // reverse is positive


    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================

    public Shooter() {

        // initialize motor 11, 12, and 13 as a SparkMax motor
        m_motor_11 = new SparkMax(11, MotorType.kBrushless); // feeder motor
        m_motor_12 = new SparkMax(12, MotorType.kBrushless); // shooter motor 1 (leader)
        m_motor_13 = new SparkMax(13, MotorType.kBrushless); // shooter motor 2 (follower)

        // get encoders for leader shooter motor (motor 12)
        m_encoder_12 = m_motor_12.getEncoder();

        // FEEDER MOTOR CONFIG (motor 11)
        SparkMaxConfig motor_11_config = new SparkMaxConfig();
        motor_11_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // LEADER MOTOR CONFIG (motor 12)
        SparkMaxConfig motor_12_config = new SparkMaxConfig();
        motor_12_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor_12_config.encoder
            .positionConversionFactor(1.0)      // 1 rotation = 1.0 units
            .velocityConversionFactor(1.0);     // 1 RPM = 1.0 units

        // FOLLOWER MOTOR CONFIG (motor 13)
        SparkMaxConfig motor_13_config = new SparkMaxConfig();
        motor_13_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor_13_config.follow(12, true); // false = same direction, true = opposite direction if motors are mirrored (TEST THIS BEFORE DEPLOYING)

        //m_motor_11.configure(motor_hyy11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_12.configure(motor_12_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_13.configure(motor_13_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // ==================== SHOOTER METHODS ====================

    // Set shooter motor speeds (change during testing)
    // private void forwardShooter() {
    //     m_motor_12.set(SHOOTER_SPEED_FORWARD); // motor 13 will automatically follow
    // }
    // private void reverseShooter() {
    //     m_motor_12.set(SHOOTER_SPEED_REVERSE);      // motor 13 will automatically follow
    // }
    // private void stopShooter() {
    //     m_motor_12.set(0);   // motor 13 will automatically follow
    // }

    // Get current shooter velocity in RPM
    public double getShooterVelocityRPM() {
        return m_encoder_12.getVelocity();  // Returns RPM, only read leader (motor 12)
    }

    // Check if shooter is at target speed and ready to shoot
    public boolean isShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - TARGET_SHOOTER_RPM) < SPEED_TOLERANCE_RPM;
    }

    // ==================== FEEDER + MOTOR METHODS ====================

    // Adjust speeds based on testing
    // private void forwardFeeder() {
    //     m_motor_11.set(0.5);
    // }
    private void forwardAll() {
        m_motor_12.set(SHOOTER_SPEED_FORWARD); // motor 13 will automatically follow
        m_motor_11.set(FEEDER_SPEED_FORWARD);
    }
    private void reverseAll() {
        m_motor_12.set(FEEDER_SPEED_REVERSE);      // motor 13 will automatically follow
        m_motor_11.set(FEEDER_SPEED_REVERSE);
    }
    private void stopAll() {
        m_motor_12.set(0);
        m_motor_11.set(0);
    }

    // ==================== SHOOTER COMMANDS ====================
    
    // public Command smartFeederCommand() {
    //     return new RunCommand(() -> {
    //         if (isShooterReady()) {
    //             forwardAll();  // Run feeder when shooter at speed
    //         } else {
    //             stopAll();     // Auto-pause when shooter slows down
    //         }
    // }, this).withName("SmartFeeder");
    // }
   
    // Basic forward shooter command (can be used for testing or manual control)
    // public Command forwardShooterCommand(){
    //     return new RunCommand(this::forwardShooter, this).withName("ForwardShooter");
    // }

    // public Command reverseShooterCommand(){
    //     return new RunCommand(this::reverseShooter, this).withName("ReverseShooter");
    // }
    // public Command stopShooterCommand(){
    //     return new InstantCommand(this::stopShooter, this).withName("StopShooter");
    // }

    // ==================== FEEDER & MOTOR COMMANDS ====================

    public Command forwardAllCommand(){
        return new RunCommand(this::forwardAll, this).withName("ForwardAll");
    }
    // public Command reverseAllCommand(){
    //     return new RunCommand(this::reverseAll, this).withName("ReverseAll");
    // }
    public Command stopAllCommand(){
        return new InstantCommand(this::stopAll, this).withName("StopAll");
    }

    @Override
    public void periodic() {
        // Log shooter speed to dashboard for monitoring
        SmartDashboard.putNumber("Shooter/Current RPM", getShooterVelocityRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", TARGET_SHOOTER_RPM);
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady());
        //SmartDashboard.putBoolean("Shooter/Feeder Running", m_motor_11.get() > 0.1); 
    }
}