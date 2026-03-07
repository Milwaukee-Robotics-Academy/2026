package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    
    private SparkMax m_motor_11; // feeder motor
    private SparkMax m_motor_12; // shooter motor 1 (leader)
    private SparkMax m_motor_13; // shooter motor 2 (follower)

    private RelativeEncoder m_encoder_12;                            // encoder motor 1 (only for leader motor, since follower will mirror it)
    private SparkClosedLoopController m_PID_12;
    private ClosedLoopSlot m_PID_far = ClosedLoopSlot.kSlot0;      // FAR range: PID slot 0 for shooter motor 12
    private ClosedLoopSlot m_PID_close = ClosedLoopSlot.kSlot1;    // CLOSE range: PID slot 1 for shooter motor 12

    // Target shooter speed and tolerance (adjust as needed based on testing)
    private static final double FAR_TARGET_SHOOTER_RPM = 4000.0;  // Adjust this!
    private static final double FAR_SPEED_TOLERANCE_RPM = 150.0;  // Within 150 RPM = ready

    private static final double CLOSE_TARGET_SHOOTER_RPM = 2000.0;  // Adjust this!
    private static final double CLOSE_SPEED_TOLERANCE_RPM = 150.0;  // Within 150 RPM = ready 

    //private static final double SHOOTER_SPEED_FORWARD = -0.75;       // negative = forward
    //private static final double SHOOTER_SPEED_REVERSE = 0.5;         // positive = reverse

    private static final double FEEDER_SPEED_FORWARD = -0.5;         // negative = forward
    private static final double FEEDER_SPEED_REVERSE = 0.5;          // positive = reverse


    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================

    public Shooter() {

        // initialize motor 11, 12, and 13 as a SparkMax motor
        m_motor_11 = new SparkMax(11, MotorType.kBrushless); // feeder motor
        m_motor_12 = new SparkMax(12, MotorType.kBrushless); // shooter motor 1 (leader)
        m_motor_13 = new SparkMax(13, MotorType.kBrushless); // shooter motor 2 (follower)

        // get encoder for leader shooter motor (motor 12)
        m_encoder_12 = m_motor_12.getEncoder();
        m_PID_12 = m_motor_12.getClosedLoopController();

        // FEEDER MOTOR CONFIG (motor 11)
        SparkMaxConfig motor_11_config = new SparkMaxConfig();
        motor_11_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // LEADER MOTOR CONFIG (motor 12)
        SparkMaxConfig motor_12_config = new SparkMaxConfig();
        motor_12_config
            .inverted(true)                    // Invert motor 12, so spins correct direction for shooting
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor_12_config.encoder
            .positionConversionFactor(1.0)      // 1 rotation = 1.0 units
            .velocityConversionFactor(1.0);     // 1 RPM = 1.0 units
        
        
        // Documentation for closed loop config: https://docs.revrobotics.com/spark-max/operating-modes/closed-loop-control 
        // Configure for FAR shooting (slot 0)
        motor_12_config.closedLoop
            .pid(0.0002, 0.0, 0.0, m_PID_far)       // PID for far shooting, start with small P and tune later
            .outputRange(-1.0, 1.0)             // Full power range
            .feedForward.kV(0.00225, m_PID_far);       // FAR Shooting: kV = 1 / max RPM (1 represents full power)        
        
        // Configure for CLOSE shooting (slot 1)
        motor_12_config.closedLoop
            .pid(0.0002, 0.0, 0.0, m_PID_close)     // PID for close shooting, can be same as far or tuned separately
            .outputRange(-1.0, 1.0)             // Full power range
            .feedForward.kV(0.0002, m_PID_close);      // CLOSE SHooting: kV = 1 / max RPM (1 represents full power)

        // FOLLOWER MOTOR CONFIG (motor 13)
        SparkMaxConfig motor_13_config = new SparkMaxConfig();
        motor_13_config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor_13_config.follow(12, true);     // false = same direction, true = opposite direction if motors are mirrored

        m_motor_11.configure(motor_11_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_12.configure(motor_12_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_13.configure(motor_13_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== SHOOTER ONLY METHODS ====================

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
    public boolean isFarShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - FAR_TARGET_SHOOTER_RPM) < FAR_SPEED_TOLERANCE_RPM;
    }

    public boolean isCloseShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - CLOSE_TARGET_SHOOTER_RPM) < CLOSE_SPEED_TOLERANCE_RPM;
    }

    // ==================== FEEDER + MOTOR METHODS ====================

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

    // Set shooter speed using closed-loop control to reach target RPM
    private void spinUpFarShooter() {
        m_PID_12.setSetpoint(FAR_TARGET_SHOOTER_RPM, ControlType.kVelocity, m_PID_far);               
    }
    private void spinUpCloseShooter() {
        m_PID_12.setSetpoint(CLOSE_TARGET_SHOOTER_RPM, ControlType.kVelocity, m_PID_close);               
    }
    private void stopShooter() {
        m_motor_12.set(0);
    }

    // Spin up shooter and run feeder only when shooter is at speed
    private void shootFarSequence() {
        spinUpFarShooter();
        if (isFarShooterReady()) {
            forwardFeeder();
        } else {
            stopFeeder();
        }
    }

    private void shootCloseSequence() {
        spinUpCloseShooter();
        if (isCloseShooterReady()) {
            forwardFeeder();
        } else {
            stopFeeder();
        }
    }

    // Turn off shooter and feeder
    private void stopAll() {
        stopShooter();
        stopFeeder();
    }

    // ==================== SHOOTER ONLY COMMANDS ====================
   
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

    public Command shootCloseSequenceCommand() {
        return new RunCommand(this::shootCloseSequence, this).withName("ShootCloseSequence");
    }

    public Command shootFarSequenceCommand() {
        return new RunCommand(this::shootFarSequence, this).withName("ShootFarSequence");
    }

    public Command stopAllCommand() {
        return new RunCommand(this::stopAll, this).withName("StopAll");
    }

    @Override
    public void periodic() {
    // Log shooter speed to dashboard for monitoring
    //SmartDashboard.putBoolean("Shooter/Feeder Running", m_motor_11.get() > 0.1); 

    // Current velocity
    SmartDashboard.putNumber("Shooter/RPM", getShooterVelocityRPM());
    
    // Ready indicators
    SmartDashboard.putBoolean("Shooter/Far Ready", isFarShooterReady());
    SmartDashboard.putBoolean("Shooter/Close Ready", isCloseShooterReady());
    
    // Target speeds (helpful to see what you're aiming for)
    SmartDashboard.putNumber("Shooter/Far Target RPM", FAR_TARGET_SHOOTER_RPM);
    SmartDashboard.putNumber("Shooter/Close Target RPM", CLOSE_TARGET_SHOOTER_RPM);
    }
}