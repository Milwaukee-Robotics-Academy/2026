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
    
    private SparkMax m_motor_12; // shooter motor 1 (leader)
    private SparkMax m_motor_13; // shooter motor 2 (follower)

    private RelativeEncoder m_encoder_12;                               // encoder motor 1 (only for leader motor, since follower will mirror it)
    private SparkClosedLoopController m_PID_12;
    private ClosedLoopSlot m_PID_far = ClosedLoopSlot.kSlot0;           // FAR range: PID slot 0 for shooter motor 12
    private ClosedLoopSlot m_PID_close = ClosedLoopSlot.kSlot1;         // CLOSE range: PID slot 1 for shooter motor 12

    // Target shooter speed and tolerance (adjust as needed based on testing)
    private static final double FAR_TARGET_SHOOTER_RPM = 4000.0;        // Adjust this!
    private static final double FAR_SPEED_TOLERANCE_RPM = 150.0;        // Within 150 RPM = ready

    private static final double CLOSE_TARGET_SHOOTER_RPM = 2000.0;      // Adjust this!
    private static final double CLOSE_SPEED_TOLERANCE_RPM = 150.0;      // Within 150 RPM = ready 

    //private static final double SHOOTER_SPEED_FORWARD = -0.75;        // negative = forward
    //private static final double SHOOTER_SPEED_REVERSE = 0.5;          // positive = reverse

    private boolean m_isFarMode = true;                                 // Default to far shooting

    // ==================== CONSTRUCTOR (CONFIGURE MOTORS) ====================

    public Shooter() {

        // initialize motors
        m_motor_12 = new SparkMax(12, MotorType.kBrushless); // shooter motor 1 (leader)
        m_motor_13 = new SparkMax(13, MotorType.kBrushless); // shooter motor 2 (follower)

        // get encoder for leader shooter motor (motor 12)
        m_encoder_12 = m_motor_12.getEncoder();
        m_PID_12 = m_motor_12.getClosedLoopController();
        
        // LEADER MOTOR CONFIG (motor 12)
        SparkMaxConfig motor_12_config = new SparkMaxConfig();
        motor_12_config
            .inverted(true)                    // Invert motor 12, so spins correct direction for shooting
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        motor_12_config.encoder
            .positionConversionFactor(1.0)      // 1 rotation = 1.0 units
            .velocityConversionFactor(1.0);     // 1 RPM = 1.0 units
        
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

        m_motor_12.configure(motor_12_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor_13.configure(motor_13_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ==================== SHOOTER METHODS ====================

    // Get current shooter velocity in RPM
    public double getShooterVelocityRPM() {
        return m_encoder_12.getVelocity();  // Returns RPM, only read leader (motor 12)
    }

    public boolean isFarShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - FAR_TARGET_SHOOTER_RPM) < FAR_SPEED_TOLERANCE_RPM;
    }

    public boolean isCloseShooterReady() {
        double currentRPM = getShooterVelocityRPM();
        return Math.abs(currentRPM - CLOSE_TARGET_SHOOTER_RPM) < CLOSE_SPEED_TOLERANCE_RPM;
    }

    public boolean isShooterReady() {
        if (m_isFarMode) {
            return isFarShooterReady();
        } else {
            return isCloseShooterReady();
        }   
    }

    // ==================== SET MOTOR SPEEDS ====================

    // Set shooter speed using closed-loop control to reach target RPM
    private void spinUpFarShooter() {
        m_isFarMode = true; 
        m_PID_12.setSetpoint(FAR_TARGET_SHOOTER_RPM, ControlType.kVelocity, m_PID_far);               
    }
    private void spinUpCloseShooter() {
        m_isFarMode = false; 
        m_PID_12.setSetpoint(CLOSE_TARGET_SHOOTER_RPM, ControlType.kVelocity, m_PID_close);               
    }
    private void stopShooter() {
        m_motor_12.set(0);
    }

    // ==================== SHOOTER COMMANDS ====================

    public Command spinUpCloseCommand() {
        return new RunCommand(this::spinUpCloseShooter, this).withName("SpinUpCloseShooter");
    }

    public Command spinUpFarCommand() {
        return new RunCommand(this::spinUpFarShooter, this).withName("SpinUpFarShooter");
    }

    public Command stopCommand() {
        return new RunCommand(this::stopShooter, this).withName("StopShooter");
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
    SmartDashboard.putBoolean("Shooter/Ready", isShooterReady());
    
    // Target speeds (helpful to see what you're aiming for)
    SmartDashboard.putNumber("Shooter/Far Target RPM", FAR_TARGET_SHOOTER_RPM);
    SmartDashboard.putNumber("Shooter/Close Target RPM", CLOSE_TARGET_SHOOTER_RPM);
    }
}