package frc.robot;

import edu.wpi.first.wpilibj.RobotController;

public class DriverStation {

	public static double getBatteryVoltage() {
		return RobotController.getBatteryVoltage();
	}
}