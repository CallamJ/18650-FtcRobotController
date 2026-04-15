package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;
import org.firstinspires.ftc.teamcode.utilities.Notifier;

/**
 * Represents a controllable degree of motion component, allowing position-based movement
 * using a controller. This abstract class provides the basic structure for components
 * that need to move to a target position while using a control algorithm.
 */
public abstract class AxisComponent {
	/** The control algorithm used to adjust movement. */
	protected final ControlAlgorithm controller;

	/** The target position the component is trying to reach. */
	private double target;

	/** Notifier used to signal when the component is no longer busy. Will wake waiting threads when the component goes from {@code isBusy==true} to {@code isBusy==false}*/
	public final Notifier noLongerBusyNotifier;

	/**
	 * Constructs an AxisComponent with a specified control algorithm.
	 *
	 * @param controller the controller used for this component.
	 */
	protected AxisComponent(ControlAlgorithm controller) {
		this.controller = controller;
		this.noLongerBusyNotifier = controller.getNoLongerBusyNotifier();
	}

	/**
	 * Runs a control loop cycle, updating the PID controller and checking
	 * if the movement is complete. If movement has stopped, it notifies waiting threads.
	 *
	 * @implNote This method calls {@link #tickPIDF()} to update the PID controller.
	 * If the controller reaches zero output, it notifies waiting threads.
	 */
	public void tick() {
		tickPIDF();
	}

	/**
	 * Method that must be implemented to update the PID controller
	 * and adjust the component's movement.
	 *
	 * @implSpec Subclasses must implement this method to update the PID controller
	 * and apply necessary adjustments to the component.
	 */
	protected abstract void tickPIDF();

	/**
	 * Gets the current position of the component.
	 *
	 * @return the current position.
	 * @implSpec must return an accurate guess of the actual position in the units the control algorithm is tuned for.
	 */
	public abstract double getCurrentPosition();

	/**
	 * Retrieves the target position the component is trying to reach.
	 *
	 * @return the target position.
	 */
	public double getTargetPosition() {
		return target;
	}

	/**
	 * Determines whether the component is still moving toward its target position.
	 *
	 * @return true if the component has not yet reached its target, false otherwise.
	 */
	public boolean isBusy() {
		return Math.abs(getTargetPosition() - getCurrentPosition()) >= controller.getTolerance();
	}

	/**
	 * Sets the target position for the component but does not actively move it.
	 * Will lazily move to the target position when {@link AxisComponent#tick()} is called.
	 * You must call {@link AxisComponent#tick()} often to reach the destination properly.
	 *
	 * @param position the desired target position.
	 */
	public void setTargetPosition(double position) {
		target = position;
	}

	 public double getPower(){
		return controller.result();
	 }
}
