package frc.robot.subsystems.drivetrain;

import frc.robot.utilities.MathUtils;
import frc.robot.utilities.Pair;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

public class SwerveDrivetrain {
    private static String[] FRPorts;
    private static String[] FLPorts;
    private static String[] BLPorts;
    private static String[] BRPorts;
    private static String[] Offsets;
    private static String[] Positions;
    private static String TicksPerRev;

    public enum WheelType {
        FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT
    }

    private static Map<WheelType, SwerveModule> swerveModules = new HashMap<>();

    public static void loadPorts(String pathname) {
        Properties application = new Properties();
        File offsets = new File(pathname);
        try {
            FileInputStream in = new FileInputStream(offsets);
            application.load(in);
        } catch (IOException e) {
            e.printStackTrace();
        }

        FRPorts = application.get("front_right_ports").toString().split(",");
        FLPorts = application.get("front_left_ports").toString().split(",");
        BRPorts = application.get("back_right_ports").toString().split(",");
        BLPorts = application.get("back_left_ports").toString().split(",");
        Offsets = application.get("offsets").toString().split(",");
        Positions = application.get("positions").toString().split(":");
        TicksPerRev = application.get("ticks_per_revolution").toString();

    }

    public SwerveDrivetrain() {
        swerveModules.put(WheelType.FRONT_RIGHT, new SwerveModule(Integer.parseInt(FRPorts[0]), Integer.parseInt(FRPorts[1]), Integer.parseInt(FRPorts[2]), Integer.parseInt(FRPorts[3]), Double.valueOf(Offsets[0]), Positions[0],  Integer.parseInt(TicksPerRev)));
        swerveModules.put(WheelType.BACK_RIGHT, new SwerveModule(Integer.parseInt(BRPorts[0]), Integer.parseInt(BRPorts[1]), Integer.parseInt(BRPorts[2]), Integer.parseInt(BRPorts[3]), Double.valueOf(Offsets[3]), Positions[3],  Integer.parseInt(TicksPerRev)));
        swerveModules.put(WheelType.BACK_LEFT, new SwerveModule(Integer.parseInt(BLPorts[0]), Integer.parseInt(BLPorts[1]), Integer.parseInt(BLPorts[2]), Integer.parseInt(BLPorts[3]), Double.valueOf(Offsets[2]), Positions[2],  Integer.parseInt(TicksPerRev)));
        swerveModules.put(WheelType.FRONT_LEFT, new SwerveModule(Integer.parseInt(FLPorts[0]), Integer.parseInt(FLPorts[1]), Integer.parseInt(FLPorts[2]), Integer.parseInt(FLPorts[3]), Double.valueOf(Offsets[1]), Positions[1],  Integer.parseInt(TicksPerRev)));

    }

    public void drive(Pair<Double, Double> translation, double rotation) {
        double max = 1.0;
        double radius;

        for (WheelType type : swerveModules.keySet()) {
            SwerveModule wheel = swerveModules.get(type);

            radius = MathUtils.pythagorean(wheel.getPosition().getFirst(), wheel.getPosition().getSecond());

            // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
            // swerveN.pdf
            double wxi = translation.getFirst() + rotation * wheel.getPosition().getFirst() / radius;
            double wyi = translation.getSecond() - rotation * wheel.getPosition().getSecond() / radius;
            double speed = Math.sqrt(Math.pow(wxi, 2) + Math.pow(wyi, 2));
            double angle = Math.atan2(wxi, wyi);
            wheel.setSpeedCommand(speed);
            wheel.setAngleCommand(MathUtils.convertRange(0.0, 360.0, 0.0, 1024.0, MathUtils.resolveAngle(Math.toDegrees(angle) + wheel.getOffset())));

            // find the maximum speed command for normalizing below
            if (speed > max) {
                max = speed;
            }
        }

        for (WheelType type : swerveModules.keySet()) {
            SwerveModule wheel = swerveModules.get(type);

            double speed = wheel.getSpeedCommand() / max; // normalized to maximum of 1
            wheel.setSpeedCommand(speed);
        }

        for (WheelType type : swerveModules.keySet()) {
            SwerveModule wheel = swerveModules.get(type);

            wheel.drive();
        }
    }

    public void lock() {
        swerveModules.get(WheelType.FRONT_LEFT).setDefenceMode(-90.0);
        swerveModules.get(WheelType.FRONT_RIGHT).setDefenceMode(90.0);
        swerveModules.get(WheelType.BACK_RIGHT).setDefenceMode(-90.0);
        swerveModules.get(WheelType.BACK_LEFT).setDefenceMode(90.0);
    }

    public void unlock() {
        for (WheelType type : swerveModules.keySet()) {
            SwerveModule wheel = swerveModules.get(type);
            wheel.setDefenceMode(0.0);
        }
    }
}