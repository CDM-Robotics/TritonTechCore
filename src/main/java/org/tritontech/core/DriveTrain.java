package org.tritontech.core;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.littletonrobotics.junction.Logger;

/*
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController; */

import choreo.auto.AutoFactory;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.trajectory.SwerveSample;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Field;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.util.WPIUtilJNI;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandXboxController;

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

    private Field m_kTagLayout;
    private double m_distanceCorrection;

    private boolean m_visionConstantsInitialized;
    private boolean m_visionConstantsReported;

    private boolean isLiveUpdatedOdometry;

    private ChassisVelocities targetChassisSpeeds = new ChassisVelocities();

    // Choreo parameters
    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private PIDController m_headingController;
    private final AutoFactory autoFactory;

    // SystemCore's built-in IMU (replaces the NavX, which needed the roboRIO MXP port)
    private final OnboardIMU m_gyro;
    private double m_angleOffset;
    private boolean m_gyroInverted = false;
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

    double desiredBias = 0.0;

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
            Vision p_Vision,
            SwerveSample s) {
        this(frontLeft, frontRight, rearLeft, rearRight, driveKinematics, magLimiter, rotLimiter, p_Vision, s,
                MountOrientation.FLAT);
    }

    /**
     * @param imuMountOrientation how the SystemCore is mounted on the robot, which
     *                            determines which IMU axis is used as yaw.
     */
    public DriveTrain(SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule rearLeft,
            SwerveModule rearRight,
            SwerveDriveKinematics driveKinematics,
            SlewRateLimiter magLimiter,
            SlewRateLimiter rotLimiter,
            Vision p_Vision,
            SwerveSample s,
            MountOrientation imuMountOrientation) {

        m_gyro = new OnboardIMU(imuMountOrientation);
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

        // Alliance flipping is OFF on purpose. Without vision the robot only knows its pose
        // relative to where each auto resets it, so running blue-built trajectories unflipped on
        // red executes the same path rotated 180 degrees about field center -- which is the
        // correct red path on a point-symmetric field (2026). Flipping would also put the gyro
        // heading in absolute field coordinates (~180 deg on red), making field-relative teleop
        // drive backwards for a red-side driver until the heading is re-zeroed.
        // Turn it on only once vision (AprilTags) provides an absolute field pose.
        autoFactory = new AutoFactory(
            this::getPose, // A function that returns the current robot pose
            this::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            this::followTrajectory, // The drive subsystem trajectory follower
            false, // If alliance flipping should be enabled
            this
        );
    }

    public void setDriveConfig() {

    }

    public void zeroHeading() {
        m_angleOffset = 0;
        m_gyro.resetYaw();
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

    public void setVisionConstants(Field kTagLayout, double distanceCorrection) {
        m_kTagLayout = kTagLayout;
        m_distanceCorrection = distanceCorrection;
        m_visionConstantsInitialized = true;
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        double bias = 0.0;

        if(Math.abs(rot) < 0.05) {
            double forwardThreshold = 0.01;
            if(xSpeed > forwardThreshold) {
                bias += desiredBias;
            } else if(xSpeed < -forwardThreshold) {
                bias -= desiredBias;
            }
        }

        rot += bias;

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

                return;
            }
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

        var commanded = new ChassisVelocities(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        var swerveModuleStates = m_driveKinematics.toSwerveModuleVelocities(
                fieldRelative
                        ? commanded.toRobotRelative(Rotation2d.fromDegrees(getAngle()))
                        : commanded);
        setModuleStates(swerveModuleStates);
    }

    public double getAngle() {
        // OnboardIMU yaw is CCW-positive (WPILib convention), unlike the NavX's CW-positive
        // getAngle(), so no negation is needed here.
        double yaw = m_gyroInverted ? -m_gyro.getYawRadians() : m_gyro.getYawRadians();
        return Math.toDegrees(MathUtil.angleModulus(yaw))
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

    public void setModuleStates(SwerveModuleVelocity[] requestedStates) {
        // desaturateWheelVelocities returns a new array (2026's desaturateWheelSpeeds mutated in place)
        SwerveModuleVelocity[] desiredStates = SwerveDriveKinematics.desaturateWheelVelocities(
                requestedStates, m_maxSpeedMPS);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleVelocity[] getModuleStates() {
        SwerveModuleVelocity[] states = new SwerveModuleVelocity[SwerveModules.length];
        for (int i = 0; i < SwerveModules.length; i++) {
            states[i] = SwerveModules[i].getState();
        }
        return states;
    }

    public Pose2d getPose() {
        // return m_odometry.getPoseMeters();
        Pose2d p_decompPose = m_currentOdometry.getEstimatedPosition();
        Pose2d p_Pose2d = new Pose2d(p_decompPose.getX(), p_decompPose.getY(), p_decompPose.getRotation());

        Telemetry.log("Current Odometry Pose", p_decompPose.getRotation().getDegrees());

        return p_Pose2d;
    }

    public void setHeading(double p_DegAngle) {
        m_gyro.resetYaw();
        m_angleOffset = p_DegAngle;
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] swervePos = getModulePositions();

        setHeading(pose.getRotation().getDegrees());
        double ang = getAngle();

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
        Telemetry.log("X", Units.metersToInches(getPose().getX()));
        Telemetry.log("Y", Units.metersToInches(getPose().getY()));
        Telemetry.log("Angle", getAngle());

        if(visionEst.isPresent()) {
            m_Vision.getTargetingYaw();
            Telemetry.log("DEBUG Vision Estimate Present", visionEst.isPresent());
        }

        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = m_Vision.getEstimationStdDevs();

                    est.estimatedPose.toPose2d().toString();
                    Telemetry.log("Est Targets Used (first)", est.targetsUsed.get(0).fiducialId);
                    Telemetry.log("Vision Estimate Pose2d (X)", estPose.getX());
                    Telemetry.log("Vision Estimate Pose2d (Y)", estPose.getY());
                    addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);

                    m_nearestTarget = getNearestTargetID();
                    Telemetry.log(("Nearest Target ID"), m_nearestTarget);
                    Optional<Pose3d> nearestPose3d = m_kTagLayout.getTagPose(m_nearestTarget);
                    m_nearestTargetPose = nearestPose3d;
                    if (nearestPose3d.isPresent()) {
                        double d = 0.0;
                        Translation2d t2d = new Translation2d(nearestPose3d.get().getX(), nearestPose3d.get().getY());
                        d = t2d.minus(getPose().getTranslation()).getNorm() + m_distanceCorrection;
                        Telemetry.log("Nearest Target Distance", d);
                        Telemetry.log("Nearest Target Distance(in)", Units.metersToInches(d));
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

        for (FieldTag tag : m_kTagLayout.getTags()) {
            if (minDistance == 0.0) {
                tagID = tag.getID();
                minDistance = m_currentOdometry.getEstimatedPosition().getTranslation()
                        .getDistance(tag.getPose().toPose2d().getTranslation());
            } else {
                currDistance = m_currentOdometry.getEstimatedPosition().getTranslation()
                        .getDistance(tag.getPose().toPose2d().getTranslation());
                if (minDistance > currDistance) {
                    minDistance = currDistance;
                    tagID = tag.getID();
                }
            }
        }

        return tagID;
    }

    public void followTrajectory(SwerveSample sample) {
        // Log the trajectory inputs for troubleshooting
        Logger.recordOutput("Choreo/SampleX", sample.x);
        Logger.recordOutput("Choreo/SampleY", sample.y);
        Logger.recordOutput("Choreo/SampleHeading", sample.heading);

        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Log the current position/heading for troubleshooting
        Logger.recordOutput("Drive/PoseX", pose.getX());
        Logger.recordOutput("Drive/PoseY", pose.getY());
        Logger.recordOutput("Drive/PoseHeading", pose.getRotation().getRadians());
        Logger.recordOutput("PID/XError", sample.x - pose.getX());
        Logger.recordOutput("PID/YError", sample.y - pose.getY());
        Logger.recordOutput("PID/HeadingError",
            MathUtil.angleModulus(sample.heading - pose.getRotation().getRadians())
        );

        // 1. Sum up the Field-Relative velocities
        double xPid = xController.calculate(pose.getX(), sample.x);
        double yPid = yController.calculate(pose.getY(), sample.y);
        double omegaPid = m_headingController.calculate(
            pose.getRotation().getRadians(),
            sample.heading
        );

        // Log the calculated PID values
        Logger.recordOutput("PID/XOutput", xPid);
        Logger.recordOutput("PID/YOutput", yPid);
        Logger.recordOutput("PID/OmegaOutput", omegaPid);

        double targetFieldVx = sample.vx + xPid;
        double targetFieldVy = sample.vy + yPid;
        double targetOmega = sample.omega + omegaPid;

        // 2. CONVERT Field-Relative TO Robot-Relative
        ChassisVelocities robotRelativeSpeeds = new ChassisVelocities(
            targetFieldVx,
            targetFieldVy,
            targetOmega
        ).toRobotRelative(pose.getRotation()); // Crucial: use the current robot heading

        // Log what we're sending to the modules
        Logger.recordOutput("Drive/RobotRelativeVx", robotRelativeSpeeds.vx);
        Logger.recordOutput("Drive/RobotRelativeVy", robotRelativeSpeeds.vy);
        Logger.recordOutput("Drive/RobotRelativeOmega", robotRelativeSpeeds.omega);

        // 3. Pass the ROBOT-RELATIVE speeds to your builder
        driveAutoBuilder(robotRelativeSpeeds);

        /* m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        double headingError = MathUtil.angleModulus(sample.heading - pose.getRotation().getRadians());

        ChassisVelocities speeds = new ChassisVelocities(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + m_headingController.calculate(0.0, headingError)
        );

        driveAutoBuilder(speeds); */
    }

    public void driveAutoBuilder(ChassisVelocities p_ChassisSpeed) {
        ChassisVelocities targetSpeeds = p_ChassisSpeed.discretize(0.02);
        SwerveModuleVelocity[] targetStates = m_driveKinematics.toSwerveModuleVelocities(targetSpeeds);

        setModuleStates(targetStates);
    }

    public ChassisVelocities getChassisSpeed() {
        return m_driveKinematics.toChassisVelocities(getModuleStates());
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
        Telemetry.log("Auto Approach", approach);
        autoApproach = approach;
    }

    public boolean isAutoApproach() {
        return autoApproach;
    }

    public void hardResetPose(Pose2d pose) {
        m_currentOdometry.resetPose(pose);
    }

    
    public void stopModules() {
        SwerveModuleVelocity[] stopStates = new SwerveModuleVelocity[] {
        new SwerveModuleVelocity(0.0, m_frontLeft.getState().angle),
        new SwerveModuleVelocity(0.0, m_frontRight.getState().angle),
        new SwerveModuleVelocity(0.0, m_rearLeft.getState().angle),
        new SwerveModuleVelocity(0.0, m_rearRight.getState().angle)
        };

        setModuleStates(stopStates);
    }

    public CommandXboxController getDefaultDriveController(int port, double bumperFactor, double triggerFactor) {
        CommandXboxController driver = new CommandXboxController(port);

        driver.view().onTrue(new InstantCommand(() -> this.resetThrottle()));
        driver.rightBumper().onTrue(new InstantCommand(() -> this.setDriverThrottle(bumperFactor)));
        driver.rightBumper().onFalse(new InstantCommand(() -> this.setDriverThrottle(1.0)));
        driver.rightTrigger().onTrue(new InstantCommand(() -> this.setDriverThrottle(triggerFactor)));
        driver.rightTrigger().onFalse(new InstantCommand(() -> this.setDriverThrottle(1.0)));
        driver.menu().onTrue(new InstantCommand(() -> this.zeroHeading()));

        this.setDefaultCommand(new DriveCmd(this, driver));

        return driver;
    }

    public Command buildTrajectory(String trajectory, PIDController headingController) {
        return buildTrajectory(trajectory, headingController, false);
    }

    public Command buildTrajectory(String trajectory, PIDController headingController, boolean resetPose) {
        Command resetCmd = null;
        m_headingController = headingController;
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_headingController.reset();
        headingController.reset();
        xController.reset();
        yController.reset();

        if(resetPose) {
            resetCmd = autoFactory.resetOdometry(trajectory);
        }

        Command traj = autoFactory.trajectoryCmd(trajectory);
        
        return ((resetPose && (resetCmd != null)) ? resetCmd.andThen(traj) : traj);
        
    //    return resetCmd;
    }

    /**
     * Flips the sign of the SystemCore IMU yaw. getAngle() must increase when the robot turns
     * counter-clockwise (seen from above); set this if it decreases instead, e.g. because the
     * SystemCore is mounted upside down. Call before zeroing the heading or resetting odometry.
     */
    public void setGyroInverted(boolean inverted) {
        m_gyroInverted = inverted;
    }

    public void setGyroBias(double b) {
        desiredBias = b;
    }
}
