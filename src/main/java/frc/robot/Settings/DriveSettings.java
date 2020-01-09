package frc.robot.Settings;

import frc.robot.utilities.Pair;

public class DriveSettings {
    //Order for all arrays: (FR, FL, BL, BR)

    //Setup CAN ports
    public static final int[] translationPorts = {1, 3, 5, 7};
    //RotationPorts are now on the PWM
    public static final int[] rotationPorts = {2, 4, 6, 8};
    public static final int[] potentiometerPorts = {0, 1, 2, 3};

    //Angle offset for each wheel's potentiometer
    public static final double[] offsets = {61.8, 84.3, 47.2, 329.4};

    //Physical location of each module's center relative to robot's center in inches
    public static final Pair<Double, Double> frontRightPos = new Pair<>(14.25, 9.375);
    public static final Pair<Double, Double> frontLeftPos = new Pair<>(14.25, -9.375);
    public static final Pair<Double, Double> backLeftPos = new Pair<>(-14.25, -9.375);
    public static final Pair<Double, Double> backRightPos = new Pair<>(-14.25, 9.375);
}
