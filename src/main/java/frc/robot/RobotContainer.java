package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.common.controller.Axis;
// import frc.robot.common.controller.ThrustmasterJoystick;
import frc.robot.common.controller.LogitechController;
import frc.robot.subsystems.*;

public class RobotContainer {
    private final LogitechController driveController = new LogitechController(Constants.DRIVE_CONTROLLER);
    
    private final SwerveDriveSubsystem drivetrainSubsystem = new SwerveDriveSubsystem();

    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        
        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        configureControllerLayout();
    }

    private void configureControllerLayout() {
        driveController.getLeftXAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY / 2);
        driveController.getLeftXAxis().setInverted(true);
        driveController.getLeftYAxis().setScale(SwerveDriveSubsystem.MAX_VELOCITY / 2);
        driveController.getRightXAxis().setScale(SwerveDriveSubsystem.MAX_ANGULAR_VELOCITY / 2);

        driveController.getBack().whenPressed(() -> drivetrainSubsystem.resetGyroAngle());
    }

    private Axis getDriveForwardAxis() {
        return driveController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return driveController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return driveController.getRightXAxis();
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return drivetrainSubsystem;
    }
}
