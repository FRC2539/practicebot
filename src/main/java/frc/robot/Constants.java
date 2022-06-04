package frc.robot;

public class Constants {
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 0;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 2;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 3;

    public static final int DRIVETRAIN_FRONT_RIGHT_TURN_MOTOR = 6;
    public static final int DRIVETRAIN_FRONT_LEFT_TURN_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_LEFT_TURN_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_RIGHT_TURN_MOTOR = 7;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 17;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 19;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 18;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 20;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Math.toRadians(-255.498);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Math.toRadians(-272.548);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Math.toRadians(-40.869);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Math.toRadians(-128.759);

    public static final int DRIVE_CONTROLLER = 0;
    // public static final int RIGHT_DRIVE_CONTROLLER = 1;
    // public static final int OPERATOR_CONTROLLER = 2;

    public static final double ROBOT_PERIODIC_ALLOCATION = 0.002;
	public static final double CONTROLLER_PERIOD = 0.005;

	public static final double DRIVETRAIN_PERIOD = 0.0015;
}