// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tritontech.core;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;

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
  private SwerveModuleVelocity m_desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(CANPort canBus,
      int drivingCANId,
      MotorControllerType drivingSparkType,
      MotorControllerType turningSparkType,
      int turningCANId,
      double chassisAngularOffset,
      String p_moduleChannel,
      SparkBaseConfig drivingConfig,
      SparkBaseConfig turningConfig,
      SimpleMotorFeedforward drivingFeedForward) {
    m_drivingSpark = MotorFactory.createMotor(drivingSparkType, canBus, drivingCANId, MotorType.kBrushless);
    m_turningSpark = MotorFactory.createMotor(turningSparkType, canBus, turningCANId, MotorType.kBrushless);

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
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition().get());
    m_drivingEncoder.setPosition(0);

    m_moduleChannel = p_moduleChannel;

    m_drivingFeedForward = drivingFeedForward;
  }

  /** Same as above, with both motors on {@link MotorFactory#DEFAULT_CAN_BUS}. */
  public SwerveModule(int drivingCANId,
      MotorControllerType drivingSparkType,
      MotorControllerType turningSparkType,
      int turningCANId,
      double chassisAngularOffset,
      String p_moduleChannel,
      SparkBaseConfig drivingConfig,
      SparkBaseConfig turningConfig,
      SimpleMotorFeedforward drivingFeedForward) {
    this(MotorFactory.DEFAULT_CAN_BUS, drivingCANId, drivingSparkType, turningSparkType, turningCANId,
        chassisAngularOffset, p_moduleChannel, drivingConfig, turningConfig, drivingFeedForward);
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
  public SwerveModuleVelocity getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleVelocity(m_drivingEncoder.getVelocity().get(),
        new Rotation2d(m_turningEncoder.getPosition().get() - m_chassisAngularOffset));
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
        m_drivingEncoder.getPosition().get(),
        new Rotation2d(m_turningEncoder.getPosition().get() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleVelocity desiredState) {
    // Apply chassis angular offset to the desired state.
    // Optimize the reference state to avoid spinning further than 90 degrees.
    // optimize() returns a new object (2026's SwerveModuleState.optimize mutated in place).
    SwerveModuleVelocity correctedDesiredState = new SwerveModuleVelocity(
        desiredState.velocity,
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)))
        .optimize(new Rotation2d(m_turningEncoder.getPosition().get()));

    // double AccelerationThingy = (optimizedDesiredState.speedMetersPerSecond -
    // m_previousVelocity)* ModuleConstants.kPAcceleration;

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // m_drivingPIDController.setReference((optimizedDesiredState.speedMetersPerSecond
    // + AccelerationThingy), CANSparkMax.ControlType.kVelocity);
    // m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
    // CANSparkMax.ControlType.kPosition);
    m_drivingClosedLoopController.setSetpoint((correctedDesiredState.velocity),
        SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0,
        m_drivingFeedForward.calculate(correctedDesiredState.velocity));
    
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(),
        SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void telemetry() {
    Telemetry.log(m_moduleChannel + "Desired Velocity",
        Math.abs(Units.metersToInches(m_desiredState.velocity)));
    Telemetry.log(m_moduleChannel + "Velocity",
        Math.abs(Units.metersToInches(m_drivingEncoder.getVelocity().get())));
    Telemetry.log(m_moduleChannel + "Drive Angle", m_turningEncoder.getPosition().get());
    Telemetry.log(m_moduleChannel + "Desired Drive Angle", m_desiredState.angle.getDegrees());

  }

  /*
   * public void showPID(){
   * Telemetry.log(m_moduleChannel + "P",
   * m_drivingPIDController.getP());
   * Telemetry.log(m_moduleChannel + "I",
   * m_drivingPIDController.getI());
   * Telemetry.log(m_moduleChannel + "D",
   * m_drivingPIDController.getD());
   * Telemetry.log(m_moduleChannel + "FF",
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
    return m_drivingEncoder.getVelocity().get();
  }
}
