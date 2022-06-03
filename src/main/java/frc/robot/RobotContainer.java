package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.common.controller.Axis;
import frc.robot.common.controller.ThrustmasterJoystick;
import frc.robot.common.controller.LogitechController;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final ThrustmasterJoystick leftDriveController = new ThrustmasterJoystick(Constants.LEFT_DRIVE_CONTROLLER);
    private final ThrustmasterJoystick rightDriveController = new ThrustmasterJoystick(Constants.RIGHT_DRIVE_CONTROLLER);
    private final LogitechController operatorController = new LogitechController(Constants.OPERATOR_CONTROLLER);
    
    private final SwerveDriveSubsystem drivetrainSubsystem = new SwerveDriveSubsystem();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        
        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        leftDriveController.getXAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY);
        leftDriveController.getYAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY);
        rightDriveController.getXAxis().setScale(SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY);

        leftDriveController.getLeftTopLeft().whenPressed(() -> drivetrainSubsystem.resetGyroAngle());
    }

    private Axis getDriveForwardAxis() {
        return leftDriveController.getYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return leftDriveController.getXAxis();
    }

    private Axis getDriveRotationAxis() {
        return rightDriveController.getXAxis();
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return drivetrainSubsystem;
    }
}
