package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.team2539.cougarswervelib.Mk3ModuleConfiguration;
import com.team2539.cougarswervelib.Mk3SwerveModuleHelper;
import com.team2539.cougarswervelib.SdsModuleConfigurations;
import com.team2539.cougarswervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.common.control.SwerveDriveSignal;
import frc.robot.util.FieldGraphics;
import frc.robot.util.Updatable;

public class SwerveDriveSubsystem extends NetworkTablesSubsystem implements Updatable {
    // Measured in meters (ask CAD dept. for this information in new robots)
    public static final double TRACKWIDTH = 0.5969;
    public static final double WHEELBASE = 0.5969;

    public static final double MAX_VOLTAGE = 12.0;

    public static final double TEMPERATURE_LOG_PERIOD = 1;

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front left
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Front right
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Back left
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back right
            );

    public static final double MAX_VELOCITY = 6380.0
            / 60
            * SdsModuleConfigurations.MK3_FAST.getDriveReduction()
            * SdsModuleConfigurations.MK3_FAST.getWheelDiameter()
            * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

    private SwerveModule[] modules;

    private final AHRS gyroscope = new AHRS();

    private final SwerveDriveOdometry swerveOdometry =
            new SwerveDriveOdometry(swerveKinematics, new Rotation2d(), new Pose2d());

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private SwerveDriveSignal driveSignal = null;

    private FieldGraphics dashboardField2d = new FieldGraphics();

    private NetworkTableEntry odometryXEntry;
    private NetworkTableEntry odometryYEntry;
    private NetworkTableEntry odometryAngleEntry;

    private NetworkTableEntry navXAngleEntry;

    private NetworkTableEntry driveTemperaturesEntry;
    private NetworkTableEntry steerTemperaturesEntry;

    private NetworkTableEntry temperaturesLogEntry;

    private Timer temperaturesLogTimer = new Timer();

    public SwerveDriveSubsystem() {
        super("Swerve Drive");

        Mk3ModuleConfiguration invertedConfiguration = new Mk3ModuleConfiguration();

        invertedConfiguration.setDriveInverted(true);

        SwerveModule frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                Mk3SwerveModuleHelper.GearRatio.FAST,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET);
        SwerveModule frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                invertedConfiguration,
                Mk3SwerveModuleHelper.GearRatio.FAST,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET);
        SwerveModule backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                Mk3SwerveModuleHelper.GearRatio.FAST,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET);
        SwerveModule backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                invertedConfiguration,
                Mk3SwerveModuleHelper.GearRatio.FAST,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_TURN_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET);

        modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        odometryXEntry = getEntry("X");
        odometryYEntry = getEntry("Y");
        odometryAngleEntry = getEntry("Angle");

        navXAngleEntry = getEntry("NavX Angle");

        driveTemperaturesEntry = getEntry("Drive Temperatures");
        steerTemperaturesEntry = getEntry("Steer Temperatures");

        temperaturesLogEntry = getEntry("Temperatures Log");
        temperaturesLogEntry.setString("time,drive 0,steer 0,drive 1,steer 1,drive 2,steer 2,drive 3,steer 3\n");

        temperaturesLogTimer.start();

        SmartDashboard.putData("Field", dashboardField2d);
    }

    public Pose2d getPose() {
        return pose;
    }

    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    public Rotation2d getGyroRotation2d() {
        return gyroscope.getRotation2d();
    }

    public double getGyroAngle() {
        return gyroscope.getAngle() % 360;
    }

    public double getRawGyroAngle() {
        return gyroscope.getAngle();
    }

    public void drive(ChassisSpeeds velocity, boolean isFieldOriented) {
        driveSignal = new SwerveDriveSignal(velocity, isFieldOriented);
    }

    public void stop() {
        driveSignal = new SwerveDriveSignal();
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        swerveOdometry.resetPosition(pose, getGyroRotation2d());
    }

    public void resetGyroAngle(Rotation2d angle) {
        gyroscope.reset();
        gyroscope.setAngleAdjustment(angle.getDegrees());
    }

    public void resetGyroAngle() {
        resetGyroAngle(Rotation2d.fromDegrees(180));
    }

    private void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleStates[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        this.velocity = swerveKinematics.toChassisSpeeds(moduleStates);
        this.pose = swerveOdometry.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation2d(), moduleStates);
    }

    private void updateModules(SwerveDriveSignal driveSignal) {
        ChassisSpeeds chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisSpeeds();
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.vxMetersPerSecond,
                    driveSignal.vyMetersPerSecond,
                    driveSignal.omegaRadiansPerSecond,
                    getGyroRotation2d());
        } else {
            chassisVelocity = new ChassisSpeeds(
                    driveSignal.vxMetersPerSecond, driveSignal.vyMetersPerSecond, driveSignal.omegaRadiansPerSecond);
        }

        if (chassisVelocity.vxMetersPerSecond == 0
                && chassisVelocity.vyMetersPerSecond == 0
                && chassisVelocity.omegaRadiansPerSecond == 0) {
            stopModules();
            return;
        }

        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(chassisVelocity);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_VELOCITY);
        for (int i = 0; i < moduleStates.length; i++) {
            var module = modules[i];
            module.set(
                    moduleStates[i].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE,
                    moduleStates[i].angle.getRadians());
        }
    }

    public void stopModules() {
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.set(0, module.getSteerAngle());
        }
    }

    @Override
    public void update() {
        updateOdometry();

        updateModules(driveSignal);
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();

        odometryXEntry.setDouble(pose.getX());
        odometryYEntry.setDouble(pose.getY());
        odometryAngleEntry.setDouble(pose.getRotation().getDegrees());

        navXAngleEntry.setDouble(getGyroRotation2d().getDegrees());

        driveTemperaturesEntry.setDoubleArray(new double[] {
            modules[0].getDriveTemperature(),
            modules[1].getDriveTemperature(),
            modules[2].getDriveTemperature(),
            modules[3].getDriveTemperature()
        });

        steerTemperaturesEntry.setDoubleArray(new double[] {
            modules[0].getSteerTemperature(),
            modules[1].getSteerTemperature(),
            modules[2].getSteerTemperature(),
            modules[3].getSteerTemperature()
        });

        if (temperaturesLogTimer.advanceIfElapsed(TEMPERATURE_LOG_PERIOD)) {
            String current = temperaturesLogEntry.getString(
                    "time,drive 0,steer 0,drive 1,steer 1,drive 2,steer 2,drive 3,steer 3\n");
            current += Timer.getFPGATimestamp() + ",";
            for (SwerveModule module : modules) {
                current += module.getDriveTemperature() + ",";
                current += module.getSteerTemperature() + ",";
            }
            current = current.substring(0, current.length() - 1) + ";";
            temperaturesLogEntry.setString(current);
        }

        dashboardField2d.setRobotPose(swerveOdometry.getPoseMeters());

        dashboardField2d.drawCircle(
                "goal",
                1,
                FieldGraphics.FIELD_WIDTH / 2 + Math.cos(Timer.getFPGATimestamp()),
                FieldGraphics.FIELD_HEIGHT / 2 + Math.sin(Timer.getFPGATimestamp()));
    }
}
