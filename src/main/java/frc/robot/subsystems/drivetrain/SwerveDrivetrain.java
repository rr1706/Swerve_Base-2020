package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.Pair;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.Settings.DriveSettings.*;

public class SwerveDrivetrain {

    public enum WheelType {
        FRONT_RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT
    }

    private static Map<WheelType, SwerveModule> swerveModules = new HashMap<>();

    public SwerveDrivetrain() {
        swerveModules.put(WheelType.FRONT_RIGHT, new SwerveModule(translationPorts[0], rotationPorts[0], potentiometerPorts[0], offsets[0], frontRightPos));
        swerveModules.put(WheelType.FRONT_LEFT, new SwerveModule(translationPorts[1], rotationPorts[1], potentiometerPorts[1], offsets[1], frontLeftPos));
        swerveModules.put(WheelType.BACK_LEFT, new SwerveModule(translationPorts[2], rotationPorts[2], potentiometerPorts[2], offsets[2], backLeftPos));
        swerveModules.put(WheelType.BACK_RIGHT, new SwerveModule(translationPorts[3], rotationPorts[3], potentiometerPorts[3], offsets[3], backRightPos));
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
            wheel.setAngleCommand(MathUtils.resolveAngle(Math.toDegrees(angle) + wheel.getOffset()));

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
    public void printTest() {
        SmartDashboard.putNumber("Front Right Angle", swerveModules.get(WheelType.FRONT_RIGHT).getAngle());
        SmartDashboard.putNumber("Front Left Angle", swerveModules.get(WheelType.FRONT_LEFT).getAngle());
        SmartDashboard.putNumber("Back Left Angle", swerveModules.get(WheelType.BACK_LEFT).getAngle());
        SmartDashboard.putNumber("Back Right Angle", swerveModules.get(WheelType.BACK_RIGHT).getAngle());
    }

    public void setTest(double rotationCommand, double translationCommand)
    {
        swerveModules.get(WheelType.FRONT_RIGHT).setTestRotationMotor(rotationCommand);
        swerveModules.get(WheelType.FRONT_RIGHT).setTestTranslationMotor(translationCommand);
    }

}