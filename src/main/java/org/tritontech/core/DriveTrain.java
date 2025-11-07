package org.tritontech.core;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveTrain extends SubsystemBase {

    static {
        VersionManager.initialize(); // Triggers VersionManager's static block
    }

    private SwerveSample samp;

    // Create MAXSwerveModules
    SwerveModule[] SwerveModules;
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_rearLeft;
    private SwerveModule m_rearRight;

    private SwerveDrivePoseEstimator m_currentOdometry;
    private SwerveDrivePoseEstimator m_measuredOdometry;

    private boolean m_driveConstantsInitialized;
    private boolean m_driveConstantsReported;
    private double m_directionSlewRate;
    private double m_maxSpeedMPS;
    private double m_maxAngularSpeed;
    private SwerveDriveKinematics m_driveKinematics;

    private double m_trackWidth;
    private double m_bumperDistance;
    private boolean m_chassisConstantsInitialized;
    private boolean m_chassisConstantsReported;

    private AprilTagFieldLayout m_kTagLayout;
    private double m_distanceCorrection;

    private boolean m_visionConstantsInitialized;
    private boolean m_visionConstantsReported;

    private boolean isLiveUpdatedOdometry;

    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    // private final Pigeon2 m_gyro = new Pigeon2(19,"rio");
    private final AHRS m_gyro2 = new AHRS(NavXComType.kMXP_SPI);
    private double m_angleOffset;
    private double engineerThrottle;
    private double driverThrottle;

    private double m_currentRotation = 0.0;
    private double m_currentTranslationMag = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private SlewRateLimiter m_magLimiter;
    private SlewRateLimiter m_rotLimiter;

    private Vision m_Vision;
    private boolean visionToggle = false;
    private int m_nearestTarget;
    private Optional<Pose3d> m_nearestTargetPose;
    int every = 0;
    private boolean autoApproach = false;

    Field2d field;
    Field2d fieldEst;

    // Available paths in teleop. Will select path based on alliance color.
    public enum TeleopPath {
        ID18,
        IDTBD
    }

    public void setOdometryToLiveUpdate(boolean live) {
        System.out.println("Syncing live updates");
        if (live) {
            syncOdometryToVision();
        }

        isLiveUpdatedOdometry = live;
    }

    public SwerveSample getSamp() {
        return samp;
    }

    public DriveTrain(SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule rearLeft,
            SwerveModule rearRight,
            SwerveDriveKinematics driveKinematics,
            SlewRateLimiter magLimiter,
            SlewRateLimiter rotLimiter,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            Vision p_Vision,
            SwerveSample s) {

        m_driveConstantsInitialized = false;
        m_driveConstantsReported = false;
        m_chassisConstantsInitialized = false;
        m_chassisConstantsReported = false;
        m_visionConstantsInitialized = false;
        m_visionConstantsReported = false;
        samp = s;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_driveKinematics = driveKinematics;
        m_magLimiter = magLimiter;
        m_rotLimiter = rotLimiter;

        SwerveModulePosition[] swervePos = getModulePositions();
        double ang = getAngle();

        SwerveModules = new SwerveModule[] {
                m_frontLeft,
                m_frontRight,
                m_rearLeft,
                m_rearRight
        };

        m_currentOdometry = new SwerveDrivePoseEstimator(
                m_driveKinematics,
                Rotation2d.fromDegrees(ang),
                swervePos,
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        m_measuredOdometry = new SwerveDrivePoseEstimator(
                m_driveKinematics,
                Rotation2d.fromDegrees(ang),
                swervePos,
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);

        m_Vision = null;
        m_Vision = p_Vision;
        field = new Field2d();
        fieldEst = new Field2d();
        m_nearestTargetPose = null;
        isLiveUpdatedOdometry = false;

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::driveAutoBuilder, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                    // Constants class
                            translationConstants, // Translation PID constants
                            rotationConstants // Rotation PID constants
                    ),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }

                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    public void setDriveConfig() {

    }

    public void zeroHeading() {
        // m_gyro.reset();
        m_angleOffset = 0;
        m_gyro2.reset();
        System.out.println("MRAP engaged and Driving re-zero complete"); // Match Restart Alignment Protocol (MRAP)
    }

    public void setChassisConstants(double trackWidth, double bumperDistance) {
        m_trackWidth = trackWidth;
        m_bumperDistance = bumperDistance;
        m_chassisConstantsInitialized = true;
    }

    public void setDriveConstants(double directionSlewRate, double maxSpeedMPS, double maxAngularSpeed) {
        m_directionSlewRate = directionSlewRate;
        m_maxSpeedMPS = maxSpeedMPS;
        m_maxAngularSpeed = maxAngularSpeed;
        m_driveConstantsInitialized = true;
    }

    public void setVisionConstants(AprilTagFieldLayout kTagLayout, double distanceCorrection) {
        m_kTagLayout = kTagLayout;
        m_distanceCorrection = distanceCorrection;
        m_visionConstantsInitialized = true;
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (!m_driveConstantsInitialized) {
            if (!m_driveConstantsReported) {
                System.err.println("#### WARNING:  You need to set the drive constants after instantiation. ####");
                m_driveConstantsReported = true;
            }

            return;
        }

        if (!m_chassisConstantsInitialized) {
            if (!m_chassisConstantsReported) {
                System.err.println("#### WARNING:  You need to set the chassis constants after instantiation. ####");
                m_chassisConstantsReported = true;
            }

            return;
        }

        if (m_Vision != null) {
            if (!m_visionConstantsInitialized) {
                if (!m_visionConstantsReported) {
                    System.err.println("#### WARNING:  You need to set the vision constants after instantiation. ####");
                    m_visionConstantsReported = true;
                }
            }

            return;
        }

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(m_directionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 600.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * m_maxSpeedMPS;
        double ySpeedDelivered = ySpeedCommanded * m_maxSpeedMPS;
        double rotDelivered = m_currentRotation * m_maxAngularSpeed;

        var swerveModuleStates = m_driveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        setModuleStates(swerveModuleStates);
    }

    public double getAngle() {
        return Math.toDegrees(MathUtil.angleModulus(-Rotation2d.fromDegrees(m_gyro2.getAngle()).getRadians()))
                + m_angleOffset;
    }

    /*
     * public double calculateTargetDistance() {
     * return (getPose().getTranslation().getDistance(m_currentTarget));
     * }
     * 
     * public double calculateTargetXError() {
     * return Math
     * .abs(getPose().getX() - Units.inchesToMeters(m_trackWidth / 2) -
     * m_currentTarget.getX());
     * }
     * 
     * public double calculateTargetYError() {
     * return (getPose().getY()) - Units.inchesToMeters(m_trackWidth / 2) -
     * m_currentTarget.getY();
     * }
     */

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, m_maxSpeedMPS);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[SwerveModules.length];
        for (int i = 0; i < SwerveModules.length; i++) {
            states[i] = SwerveModules[i].getState();
        }
        return states;
    }

    public Pose2d getPose() {
        // return m_odometry.getPoseMeters();
        Pose2d p_decompPose = m_currentOdometry.getEstimatedPosition();
        Pose2d p_Pose2d = new Pose2d(p_decompPose.getX(), p_decompPose.getY(), p_decompPose.getRotation());

        SmartDashboard.putNumber("Current Odometry Pose", p_decompPose.getRotation().getDegrees());

        return p_Pose2d;
    }

    public void setHeading(double p_DegAngle) {
        m_gyro2.reset();
        m_angleOffset = p_DegAngle;
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] swervePos = getModulePositions();
        double ang = getAngle();

        setHeading(pose.getRotation().getDegrees());
        m_currentOdometry.resetPosition(
                Rotation2d.fromDegrees(ang),
                swervePos,
                pose);

        m_measuredOdometry.resetPosition(
                Rotation2d.fromDegrees(ang),
                swervePos,
                pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    public void syncOdometryToVision() {
        // Get the measured pose from the measured odometry
        Pose2d measuredPose = m_measuredOdometry.getEstimatedPosition();

        // Reset the current odometry to the measured pose with its gyro rotation
        m_currentOdometry.resetTranslation(measuredPose.getTranslation());
        m_currentOdometry.resetRotation(measuredPose.getRotation());
    }

    public void resetMeasuredOdometry(Pose2d pose) {
        m_measuredOdometry.resetPose(pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        zeroHeading();
        // resetEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if (m_Vision != null) {
            visionEst = m_Vision.getEstimatedGlobalPose();
        }

        SwerveModulePosition[] swervePos = getModulePositions();
        double ang = getAngle();

        field.setRobotPose(getPose());
        m_currentOdometry.update(
                Rotation2d.fromDegrees(ang),
                swervePos);
        m_measuredOdometry.update(
                Rotation2d.fromDegrees(ang),
                swervePos);
        SmartDashboard.putNumber("X", Units.metersToInches(getPose().getX()));
        SmartDashboard.putNumber("Y", Units.metersToInches(getPose().getY()));
        SmartDashboard.putNumber("Angle", getAngle());

        if(visionEst.isPresent()) {
            m_Vision.getTargetingYaw();
            SmartDashboard.putBoolean("DEBUG Vision Estimate Present", visionEst.isPresent());
        }

        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = m_Vision.getEstimationStdDevs();

                    est.estimatedPose.toPose2d().toString();
                    SmartDashboard.putNumber("Est Targets Used (first)", est.targetsUsed.get(0).fiducialId);
                    SmartDashboard.putNumber("Vision Estimate Pose2d (X)", estPose.getX());
                    SmartDashboard.putNumber("Vision Estimate Pose2d (Y)", estPose.getY());
                    addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    m_nearestTarget = getNearestTargetID();
                    SmartDashboard.putNumber(("Nearest Target ID"), m_nearestTarget);
                    Optional<Pose3d> nearestPose3d = m_kTagLayout.getTagPose(m_nearestTarget);
                    m_nearestTargetPose = nearestPose3d;
                    if (nearestPose3d.isPresent()) {
                        double d = 0.0;
                        Translation2d t2d = new Translation2d(nearestPose3d.get().getX(), nearestPose3d.get().getY());
                        d = t2d.minus(getPose().getTranslation()).getNorm() + m_distanceCorrection;
                        SmartDashboard.putNumber("Nearest Target Distance", d);
                        SmartDashboard.putNumber("Nearest Target Distance(in)", Units.metersToInches(d));
                    }

                });
    }

    /**
     * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        m_measuredOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
        if (isLiveUpdatedOdometry) {
            m_currentOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
        }
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        m_measuredOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
        if (isLiveUpdatedOdometry) {
            m_currentOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
        }
    }

    public int getNearestTargetID() {
        int tagID = 99;
        double minDistance = 0.0;
        double currDistance = 0.0;

        for (AprilTag tag : m_kTagLayout.getTags()) {
            if (minDistance == 0.0) {
                tagID = tag.ID;
                minDistance = m_currentOdometry.getEstimatedPosition().getTranslation()
                        .getDistance(tag.pose.toPose2d().getTranslation());
            } else {
                currDistance = m_currentOdometry.getEstimatedPosition().getTranslation()
                        .getDistance(tag.pose.toPose2d().getTranslation());
                if (minDistance > currDistance) {
                    minDistance = currDistance;
                    tagID = tag.ID;
                }
            }
        }

        return tagID;
    }

    /**
     * Using the PathPlanner pathfinding algorithm, pathfind from our current
     * position to a path. Used
     * in teleop to pathfind to the start of a known path location. Requires
     * AutoPathBuilder to be
     * configured before use.
     * 
     * @param wanted_path Path we want to pathfind to. Known location in TeleopPath.
     * @return Command to follow the path that it found.
     */
    /*
     * public Command teleopPathfindTo(TeleopPath wanted_path){
     * PathPlannerPath path = null;
     * 
     * if (DriverStation.getAlliance().isPresent()){
     * switch (wanted_path) {
     * case ID18:
     * if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
     * try {
     * path = PathPlannerPath.fromPathFile("ID18");
     * } catch(Exception e) {
     * return new InstantCommand();
     * }
     * }
     * break;
     * 
     * default:
     * // no valid path to select. Do nothing
     * return new InstantCommand();
     * }
     * } else {
     * // Driver alliance not selected
     * return new InstantCommand();
     * }
     * 
     * if(path == null) {
     * return new InstantCommand();
     * }
     * 
     * // Create the constraints to use while pathfinding. The constraints defined
     * in the path will only be used for the path.
     * PathConstraints constraints = new PathConstraints(
     * 3.0, 2.0,
     * Units.degreesToRadians(360), Units.degreesToRadians(180));
     * 
     * Command pathfindingCommand = AutoBuilder.pathfindToPose(
     * new Pose2d(Units.inchesToMeters(128.25), Units.inchesToMeters(164.0),
     * Rotation2d.kZero),
     * constraints
     * );
     * return pathfindingCommand;
     * }
     */

    public void driveAutoBuilder(ChassisSpeeds p_ChassisSpeed) {
        // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(p_ChassisSpeed, 0.02);

        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(p_ChassisSpeed.vxMetersPerSecond,
                p_ChassisSpeed.vyMetersPerSecond, -p_ChassisSpeed.omegaRadiansPerSecond), 0.02);
        SwerveModuleState[] targetStates = m_driveKinematics.toSwerveModuleStates(targetSpeeds);

        setModuleStates(targetStates);
    }

    public ChassisSpeeds getChassisSpeed() {
        return m_driveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setEngineerThrottle(double t) {
        engineerThrottle = t;
    }

    public double getEngineerThrottle() {
        return engineerThrottle;
    }

    public double getDriverThrottle() {
        return driverThrottle;
    }

    public void setDriverThrottle(double t) {
        driverThrottle = t;
    }

    public void resetThrottle() // Emergency Throttle Override System (ETOS)
    {
        driverThrottle = engineerThrottle = 1.0;
        System.out.println("ETOS activated");
    }

    // Additions
    public Pose2d getNearestTargetPose() {
        boolean noPose = false;

        Optional<Pose2d> optionalPose2d;
        if (m_nearestTargetPose == null) {
            // optionalPose2d = Optional.empty();
            return null;
        }
        optionalPose2d = m_nearestTargetPose.map(Pose3d::toPose2d);
        Pose2d pose2d = optionalPose2d.orElse(getPose());

        return pose2d;
    }

    public boolean hasTargets() {
        return (m_nearestTargetPose != null);
    }

    public Pose2d getNearestTargetPoseStage() {
        Pose2d p = getNearestTargetPose();
        if (p != null) {
            return getPoseInFront(p, m_bumperDistance);
        } else {
            return null;
        }
    }

    public static Pose2d getPoseInFront(Pose2d originalPose, double distance) {
        // Get the original position and heading
        Translation2d originalTranslation = originalPose.getTranslation();
        Rotation2d heading = originalPose.getRotation();

        // Calculate the offset in the direction of the heading
        double offsetX = heading.getCos() * distance;
        double offsetY = heading.getSin() * distance;

        // New position = original position + offset
        Translation2d newTranslation = originalTranslation.plus(new Translation2d(offsetX, offsetY));

        // New pose with the same heading
        return new Pose2d(newTranslation, heading);
    }

    public void setAutoApproach(boolean approach) {
        SmartDashboard.putBoolean("Auto Approach", approach);
        autoApproach = approach;
    }

    public boolean isAutoApproach() {
        return autoApproach;
    }

    public void hardResetPose(Pose2d pose) {
        m_currentOdometry.resetPose(pose);
    }

    public CommandXboxController getDefaultDriveController(int port, double bumperFactor, double triggerFactor) {
        CommandXboxController driver = new CommandXboxController(port);

        driver.back().onTrue(new InstantCommand(() -> this.resetThrottle()));
        driver.rightBumper().onTrue(new InstantCommand(() -> this.setDriverThrottle(bumperFactor)));
        driver.rightBumper().onFalse(new InstantCommand(() -> this.setDriverThrottle(1.0)));
        driver.rightTrigger().onTrue(new InstantCommand(() -> this.setDriverThrottle(triggerFactor)));
        driver.rightTrigger().onFalse(new InstantCommand(() -> this.setDriverThrottle(1.0)));
        driver.start().onTrue(new InstantCommand(() -> this.zeroHeading()));

        this.setDefaultCommand(new DriveCmd(this, driver));

        return driver;
    }
}
