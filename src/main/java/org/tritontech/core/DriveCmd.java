package org.tritontech.core;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCmd extends Command {
    private final DriveTrain m_driveTrain;
    private final CommandXboxController m_controller;

    double deadzone = 0.1; // variable for amount of deadzone
    double y = 0; // variable for forward/backward movement
    double x = 0; // variable for side to side movement
    double rot = 0; // variable for turning mo vement
    double sideMod = 1; // variable for which side is the robot on
    boolean m_autoSlew;

    public DriveCmd(DriveTrain driveTrain, CommandXboxController driveController) {
        m_driveTrain = driveTrain;
        m_controller = driveController;
        addRequirements(m_driveTrain);

        driveTrain.setEngineerThrottle(1.0);
        driveTrain.setDriverThrottle(1.0);
    }

    @Override
    public void execute() {
        double throttle = Math.min(m_driveTrain.getEngineerThrottle(), m_driveTrain.getDriverThrottle());
        SmartDashboard.putNumber("Drive Throttle", m_driveTrain.getDriverThrottle());
        SmartDashboard.putNumber("Engineer Throttle", m_driveTrain.getEngineerThrottle());

        var ySpeed = MathUtil.applyDeadband(-m_controller.getLeftX(), deadzone) * sideMod;

        var xSpeed = MathUtil.applyDeadband(-m_controller.getLeftY(), deadzone) * sideMod;

        rot = MathUtil.applyDeadband(-m_controller.getRightX(), deadzone);
        m_autoSlew = false;

        // System.out.println(m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle()));
        m_driveTrain.drive(xSpeed * throttle, ySpeed * throttle, rot * throttle, true, m_autoSlew);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveTrain.drive(0.0, 0.0, 0.0, true, false);
    }
}
