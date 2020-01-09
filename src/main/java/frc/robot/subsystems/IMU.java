package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utilities.MathUtils;

/**
 * Communicates with navX-MXP
 */
public class IMU {
	private static AHRS ahrs;

	private double offset = 0;

	/**
	 * Connects to the navX
	 */
	public static void init() {
		try {
			/*
			 * Communicate w/navX MXP via the MXP SPI Bus. Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.
			 */
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}

	/**
	 * @return Angle in degrees
	 */
	public double getAngle() {
		return MathUtils.resolveAngle(ahrs.getYaw() + offset);
	}

	/**
	 * Resets IMU angle to 0
	 */
	public void reset(double offset) {
		ahrs.reset();
		setOffset(offset);
	}

	/**
	 * sets this.offset to offset
	 * @param offset
	 */
	public void setOffset(double offset) {
		this.offset = offset;
	}
}
