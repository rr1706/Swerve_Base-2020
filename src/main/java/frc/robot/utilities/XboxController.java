package frc.robot.utilities;

import edu.wpi.first.wpilibj.Joystick;

public class XboxController extends Joystick {
	private Joystick stick;
	private double deadband;

	public XboxController(int port) {
		super(port);
		stick = new Joystick(port);
	}

	public boolean A() {
		return stick.getRawButton(1);
	}

	public boolean B() {
		return stick.getRawButton(2);
	}

	public boolean X() {
		return stick.getRawButton(3);
	}

	public boolean Y() {
		return stick.getRawButton(4);
	}

	public boolean LB() {
		return stick.getRawButton(5);
	}

	public boolean RB() {
		return stick.getRawButton(6);
	}

	public boolean Back() {
		return stick.getRawButton(7);
	}

	public boolean Start() {
		return stick.getRawButton(8);
	}

	public boolean LStickButton() {
		return stick.getRawButton(9);
	}

	public boolean RStickButton() {
		return stick.getRawButton(10);
	}

	public double LStickX() {
		if (Math.abs(stick.getRawAxis(0)) > deadband) {
			return stick.getRawAxis(0);
		} else {
			return 0.0;
		}
	}

	public double LStickY() {
		if (Math.abs(stick.getRawAxis(1)) > deadband) {
			return stick.getRawAxis(1);
		} else {
			return 0.0;
		}
	}

	public double LTrig() {
		return stick.getRawAxis(2);
	}

	public double RTrig() {
		return stick.getRawAxis(3);
	}

	public double RStickX() {
		if (Math.abs(stick.getRawAxis(4)) > deadband) {
			return stick.getRawAxis(4);
		} else {
			return 0.0;
		}
	}

	public double RStickY() {
		if (Math.abs(stick.getRawAxis(5)) > deadband) {
			return stick.getRawAxis(5);
		} else {
			return 0.0;
		}
	}

	public int DPad() {
		return stick.getPOV();
	}

	public void setDeadband(double band) {
		this.deadband = band;
	}

	public void rumbleRight(double val) {
		stick.setRumble(RumbleType.kRightRumble, val);
	}

	public void rumbleLeft(double val) {
		stick.setRumble(RumbleType.kLeftRumble, val);
	}

	public void stopRumble() {
		stick.setRumble(RumbleType.kRightRumble, 0);
		stick.setRumble(RumbleType.kLeftRumble, 0);
	}
}
