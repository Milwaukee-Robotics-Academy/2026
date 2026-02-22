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
    
    //private SparkMax m_motor_11; // feeder motor
    private SparkMax m_motor_12; // shooter motor 1 (leader)
    private SparkMax m_motor_13; // shooter motor 2 (follower)

    private RelativeEncoder m_encoder_12; // encoder motor 1 (only for leader motor, since follower will mirror it)

    // Target shooter speed and tolerance (adjust as needed based on testing)
    private static final double TARGET_SHOOTER_RPM = 3000.0;  // Adjust this!
    private static final double SPEED_TOLERANCE_RPM = 100.0;  // Within 100 RPM = ready 


    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================

    public Shooter() {

        // initialize motor 11, 12, and 13 as a SparkMax motor
        //m_motor_11 = new SparkMax(11, MotorType.kBrushless); // feeder motor
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

        //m_motor_11.configure(motor_11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_12.configure(motor_12_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_13.configure(motor_13_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // ==================== SHOOTER METHODS ====================

    // Set shooter motor speeds (change during testing)
    private void forwardShooter() {
        m_motor_12.set(0.5); // motor 13 will automatically follow
    }
    private void reverseShooter() {
        m_motor_12.set(-0.5);      // motor 13 will automatically follow
    }
    private void stopShooter() {
        m_motor_12.set(0);   // motor 13 will automatically follow
    }

    // Get current shooter velocity in RPM
    public double getShooterVelocityRPM() {
        return m_encoder_12.getVelocity();  // Returns RPM, only read leader (motor 12)
    }

    // Check if shooter is at target speed and ready to shoot
    public boolean isShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - TARGET_SHOOTER_RPM) < SPEED_TOLERANCE_RPM;
    }

    // ==================== FEEDER METHODS ====================

    // Set feeder motor speeds (change during testing)
    // private void forwardFeeder() {
    //     m_motor_11.set(0.5);
    // }
    // private void reverseFeeder() {
    //     m_motor_11.set(-0.5);
    // }
    // private void stopFeeder() {
    //     m_motor_11.set(0);
    // }

    // ==================== SHOOTER COMMANDS ====================
    
    // public Command smartFeederCommand() {
    // return new RunCommand(() -> {
    //     if (isShooterReady()) {
    //         forwardFeeder();  // Run feeder when shooter at speed
    //     } else {
    //         stopFeeder();     // Auto-pause when shooter slows down
    //     }
    // }, this).withName("SmartFeeder");
    // }
   
    // Basic forward shooter command (can be used for testing or manual control)
    public Command forwardShooterCommand(){
        return new RunCommand(this::forwardShooter, this).withName("ForwardShooter");
    }

    // public Command reverseShooterCommand(){
    //     return new RunCommand(this::reverseShooter, this).withName("ReverseShooter");
    // }
    public Command stopShooterCommand(){
        return new RunCommand(this::stopShooter, this).withName("StopShooter");
    }

    // ==================== FEEDER COMMANDS ====================

    // public Command forwardFeederCommand(){
    //     return new RunCommand(this::forwardFeeder, this).withName("ForwardFeeder");
    // }
    // public Command reverseFeederCommand(){
    //     return new RunCommand(this::reverseFeeder, this).withName("ReverseFeeder");
    // }
    // public Command stopFeederCommand(){
    //     return new RunCommand(this::stopFeeder, this).withName("StopFeeder");
    // }

    @Override
    public void periodic() {
        // Log shooter speed to dashboard for monitoring
        SmartDashboard.putNumber("Shooter/Current RPM", getShooterVelocityRPM());
        SmartDashboard.putNumber("Shooter/Target RPM", TARGET_SHOOTER_RPM);
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady());
        //SmartDashboard.putBoolean("Shooter/Feeder Running", m_motor_11.get() > 0.1); 
    }

}