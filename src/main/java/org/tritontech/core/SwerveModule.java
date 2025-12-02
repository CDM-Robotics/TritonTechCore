// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tritontech.core;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  static {
    VersionManager.initialize(); // Triggers VersionManager's static block
  }

  // private final SparkMax m_drivingSparkMax;
  private final SparkBase m_drivingSpark;
  private final SparkBase m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private final SimpleMotorFeedforward m_drivingFeedForward;

  private String m_moduleChannel;// module channel is for telemetry

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private double m_previousVelocity;



  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId,
      MotorControllerType drivingSparkType,
      MotorControllerType turningSparkType,
      int turningCANId,
      double chassisAngularOffset,
      String p_moduleChannel,
      SparkBaseConfig drivingConfig,
      SparkBaseConfig turningConfig,
      SimpleMotorFeedforward drivingFeedForward) {
    m_drivingSpark = MotorFactory.createMotor(drivingSparkType, drivingCANId, MotorType.kBrushless);
    m_turningSpark = MotorFactory.createMotor(turningSparkType, turningCANId, MotorType.kBrushless);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    m_drivingSpark.configure(drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    m_moduleChannel = p_moduleChannel;

    m_drivingFeedForward = drivingFeedForward;
  }

  @Deprecated
  public SwerveModule(int drivingCANId,
      MotorControllerType drivingSparkType,
      int turningCANId,
      double chassisAngularOffset,
      String p_moduleChannel,
      SparkBaseConfig drivingConfig,
      SparkBaseConfig turningConfig,
      SimpleMotorFeedforward drivingFeedForward) {

        this(drivingCANId, drivingSparkType, MotorControllerType.SPARK_MAX, turningCANId, chassisAngularOffset, p_moduleChannel,
        drivingConfig, turningConfig, drivingFeedForward);
      }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // double AccelerationThingy = (optimizedDesiredState.speedMetersPerSecond -
    // m_previousVelocity)* ModuleConstants.kPAcceleration;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setReference((optimizedDesiredState.speedMetersPerSecond
    // + AccelerationThingy), CANSparkMax.ControlType.kVelocity);
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
    // CANSparkMax.ControlType.kPosition);
    m_drivingClosedLoopController.setReference((correctedDesiredState.speedMetersPerSecond),
        SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0,
        m_drivingFeedForward.calculate(correctedDesiredState.speedMetersPerSecond));
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
    m_previousVelocity = m_desiredState.speedMetersPerSecond;
  }

  public void telemetry() {
    SmartDashboard.putNumber(m_moduleChannel + "Desired Velocity",
        Math.abs(Units.metersToInches(m_desiredState.speedMetersPerSecond)));
    SmartDashboard.putNumber(m_moduleChannel + "Velocity",
        Math.abs(Units.metersToInches(m_drivingEncoder.getVelocity())));
    SmartDashboard.putNumber(m_moduleChannel + "Drive Angle", m_turningEncoder.getPosition());
    SmartDashboard.putNumber(m_moduleChannel + "Desired Drive Angle", m_desiredState.angle.getDegrees());

  }

  /*
   * public void showPID(){
   * SmartDashboard.putNumber(m_moduleChannel + "P",
   * m_drivingPIDController.getP());
   * SmartDashboard.putNumber(m_moduleChannel + "I",
   * m_drivingPIDController.getI());
   * SmartDashboard.putNumber(m_moduleChannel + "D",
   * m_drivingPIDController.getD());
   * SmartDashboard.putNumber(m_moduleChannel + "FF",
   * m_drivingPIDController.getFF());
   * }
   */
  /*
   * public void updatePID(double P, double I, double D, double FF){
   * 
   * m_drivingPIDController.setP(P);
   * m_drivingPIDController.setI(I);
   * m_drivingPIDController.setD(D);
   * m_drivingPIDController.setFF(FF);
   * 
   * }
   */

  /*
   * public double getSwerveP(){
   * return m_drivingPIDController.getP();
   * }
   * 
   * public double getSwerveI(){
   * return m_drivingPIDController.getI();
   * }
   * 
   * public double getSwerveD(){
   * return m_drivingPIDController.getD();
   * }
   * 
   * public double getSwerveFF(){
   * return m_drivingPIDController.getFF();
   * }
   */
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getVelocity() {
    return m_drivingEncoder.getVelocity();
  }
}
