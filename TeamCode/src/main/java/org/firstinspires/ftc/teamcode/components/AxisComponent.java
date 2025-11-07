package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.hardware.controllers.ControlAlgorithm;
import org.firstinspires.ftc.teamcode.utilities.ChainedFuture;
import org.firstinspires.ftc.teamcode.utilities.Notifier;
import org.firstinspires.ftc.teamcode.utilities.TaskScheduler;

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

	/** Scheduler for handling asynchronous tasks. */
	protected TaskScheduler scheduler = new TaskScheduler(4);

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

	/**
	 * Moves the component to a specified position and blocks execution until it reaches the target.
	 * Continuously updates the PID loop and telemetry.
	 *
	 * @param position the target position to move to.
	 */
	public void goToBlocking(double position) {
		target = position;
		while (isBusy() && !Thread.interrupted()) {
			tick();
			OpModeCore.simpleTick();
		}
	}

	/**
	 * Moves the component to a specified position and blocks execution until it reaches the target.
	 * Continuously updates the PID loop and telemetry.
	 *
	 * @param position the target position to move to.
	 */
	public void goToBlocking(double position, long timeoutMs) {
		ElapsedTime timeoutTimer = new ElapsedTime();
		target = position;
		while (isBusy() && !Thread.interrupted() && timeoutTimer.milliseconds() < timeoutMs) {
			tick();
			OpModeCore.simpleTick();
		}
	}

	/**
	 * Moves the component to a specified position asynchronously.
	 * Will lazily move to the target position when {@link AxisComponent#tick()} is called.
	 * <p>
	 *     Used for scheduling tasks after a move, use {@link AxisComponent#setTargetPosition(double)}
	 *     for going to a target lazily when scheduling is not required.
	 * </p>
	 *
	 * @param angle the target position to move to.
	 * @return a {@code ChainedFuture} representing the asynchronous task.
	 */
	public ChainedFuture<?> goToAsync(double angle) {
		noLongerBusyNotifier.interruptWaitingThreads();
		return scheduler.runAsync(() -> {
			target = angle;
			if (isBusy()) {
				noLongerBusyNotifier.await();
			}
		});
	}

	/**
	 * Moves the component to a specified position asynchronously.
	 * Will lazily move to the target position when {@link AxisComponent#tick()} is called.
	 * <p>
	 *     Used for scheduling tasks after a move, use {@link AxisComponent#setTargetPosition(double)}
	 *     for going to a target lazily when scheduling is not required.
	 * </p>
	 *
	 * @param angle the target position to move to.
	 * @return a {@code ChainedFuture} representing the asynchronous task.
	 */
	public ChainedFuture<?> goToAsync(double angle, long timeoutMs) {
		noLongerBusyNotifier.interruptWaitingThreads();
		return scheduler.runAsync(() -> {
			target = angle;
			if (isBusy()) {
				noLongerBusyNotifier.await(timeoutMs);
			}
		});
	}


	 public double getPower(){
		return controller.result();
	 }
}
