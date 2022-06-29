package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Arrays;

public class FieldGraphics extends Field2d {
    public static final double FIELD_HEIGHT = 8.1026;
    public static final double FIELD_WIDTH = 16.4846;

    public void drawSquare(String name, double size, double x, double y) {
        getObject(name)
                .setPoses(Arrays.asList(
                        new Pose2d(new Translation2d(x + size / 2, y + size / 2), new Rotation2d(0)),
                        new Pose2d(new Translation2d(x - size / 2, y + size / 2), new Rotation2d(0)),
                        new Pose2d(new Translation2d(x - size / 2, y - size / 2), new Rotation2d(0)),
                        new Pose2d(new Translation2d(x + size / 2, y - size / 2), new Rotation2d(0))));
    }

    public void drawCircle(String name, double size, double x, double y) {
        Pose2d[] translations = new Pose2d[8];
        for (double i = 0; i < translations.length; i++) {
            translations[(int) i] = new Pose2d(
                    new Translation2d(
                            x + Math.cos(i / translations.length * Math.PI * 2) * size / 2,
                            y + Math.sin(i / translations.length * Math.PI * 2) * size / 2),
                    new Rotation2d(0));
        }

        getObject(name).setPoses(Arrays.asList(translations));
    }
}
