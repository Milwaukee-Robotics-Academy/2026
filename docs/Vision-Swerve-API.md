Vision + Swerve Subsystems — Developer Summary

Purpose
- Provide a compact reference for the Vision and Swerve subsystems used in the robot project.
- Highlight primary public methods, typical usage patterns, simulation notes, and common pitfalls.

Files
- src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java
- src/main/java/frc/robot/subsystems/swervedrive/Vision.java

SwerveSubsystem — key public API
- Constructors
  - SwerveSubsystem(File directory)
    - Create a SwerveDrive using configuration files in `directory`.
  - SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
    - Create a SwerveDrive using explicit configuration objects.

- Pose / odometry
  - getPose(): Pose2d — current odometry pose
  - resetOdometry(Pose2d pose): void — set odometry to a pose
  - zeroGyro(): void — reset gyro angle and odometry heading
  - zeroGyroWithAlliance(): void — zero gyro and adjust pose for red alliance

- Drive commands (returns Commands or direct control)
  - drive(Translation2d translation, double rotation, boolean fieldRelative): void
  - drive(ChassisSpeeds velocity): void
  - driveFieldOriented(ChassisSpeeds velocity): void
  - driveFieldOriented(Supplier<ChassisSpeeds> velocity): Command
  - driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier ang): Command
  - driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier hx, DoubleSupplier hy): Command
  - driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds>): Command
  - driveToPose(Pose2d): Command — pathfinding to a target pose (PathPlanner)
  - getAutonomousCommand(String pathName): Command — PathPlanner auto command

- Actions / Utilities
  - centerModulesCommand(): Command — center all module angles
  - lock(): void — lock wheels to prevent movement
  - setMotorBrake(boolean): void — set motor brake/coast
  - replaceSwerveModuleFeedforward(kS,kV,kA): void — replace feedforward values
  - postTrajectory(Trajectory): void — publish trajectory to Field2d

- Sensor / state
  - getHeading(): Rotation2d
  - getPitch(): Rotation2d
  - getFieldVelocity(), getRobotVelocity(): ChassisSpeeds
  - getSwerveController(), getSwerveDriveConfiguration(), getSwerveDrive(): access underlying objects

Vision — key public API
- Constructor
  - Vision(Supplier<Pose2d> currentPose, Field2d field)
    - currentPose: supplier used to read current odometry pose
    - field: Field2d to publish debug overlays

- Utilities
  - getAprilTagPose(int aprilTag, Transform2d robotOffset): Pose2d
    - Returns field pose for an april tag adjusted by robot offset. Throws if tag not found.
  - updatePoseEstimation(SwerveDrive swerveDrive): void
    - Pulls latest vision estimates and applies them to swerveDrive via addVisionMeasurement
  - getEstimatedGlobalPose(Cameras camera): Optional<EstimatedRobotPose>
  - getDistanceFromAprilTag(int id): double — distance to april tag (meters), -1 when unknown
  - getTargetFromId(int id, Cameras camera): PhotonTrackedTarget — find specific tag in camera cache
  - updateVisionField(): void — writes detected tag poses to the Field2d for debugging
  - getVisionSim(): VisionSystemSim — only when running in simulation

Cameras enum (per-camera helpers)
- addToVisionSim(VisionSystemSim systemSim): void — register simulated camera
- getBestResult(): Optional<PhotonPipelineResult> — cached best (least ambiguous) result
- getLatestResult(): Optional<PhotonPipelineResult> — cached latest result
- getEstimatedGlobalPose(): Optional<EstimatedRobotPose> — estimated pose from this camera (updates cache)

Usage examples
- Apply vision updates to odometry in periodic() (if using manual vision updates):
  swerveSubsystem.getSwerveDrive().addVisionMeasurement(pose2d, timestampSeconds);

- Run a field-relative drive command:
  Command driveCmd = swerveSubsystem.driveFieldOriented(() -> new ChassisSpeeds(x, y, omega));

- Create and run a PathPlanner auto:
  Command auto = swerveSubsystem.getAutonomousCommand("MyPath");

Simulation notes
- Both systems include simulation hooks. In simulation:
  - Vision will create a VisionSystemSim and PhotonCameraSim instances for each camera.
  - SwerveDrive may expose simulated drivetrain pose via getSimulationDriveTrainPose() — Vision uses this to update the sim.
  - Use updateVisionField() to draw detected targets on the Field2d.

Common pitfalls
- When resetting odometry, ensure gyro and module encoders match the expected heading or call resetOdometry after resetting them.
- Vision estimates can be noisy — use the provided standard-deviation heuristics and consider gating by ambiguity.
- PathPlanner configuration is read from GUI settings; missing or invalid GUI config can cause exceptions during setupPathPlanner().

Contact
- For questions about tuning or behavior, check the `SwerveDrive` implementation in `swervelib` and the PathPlanner docs used in this project.

