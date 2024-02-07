package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class DualJoystick extends GenericHID {

	private Joystick right;
	private Joystick left;

	public DualJoystick(int rightPort, int leftPort) {
		super(0);
		this.right = new Joystick(rightPort);
		this.left = new Joystick(leftPort);

	}

	public Joystick getRighJoystick() {
		return right;
	}

	public double getRightRawAxis(int axis) {
		return this.calculateDeadzone(right.getRawAxis(axis));
	}

	public double getLeftRawAxis(int axis) {
		return this.calculateDeadzone(left.getRawAxis(axis));

	}

	private double calculateDeadzone(double speed) {
		return Math.abs(speed) > Constants.DualJoystick.DEADZONE ? speed : 0;
	}

	public double getSlowMode() {
		return (this.getLeftRawAxis(Constants.DualJoystick.DIAL) * -1 + 1) / 2;
	}

	public double getAirplaneSpeed() {
		return (this.getRightRawAxis(Constants.DualJoystick.DIAL) * -1 + 1) / 2;
	}
}