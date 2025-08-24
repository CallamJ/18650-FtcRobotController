package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

/** @noinspection BusyWait*/
public class Await {

	/**
	 * Blocks the current thread until the given condition evaluates to true, polling at a fixed interval.
	 *
	 * @param condition   The condition to wait for.
	 * @param pollingRateMs The interval (in milliseconds) to check the condition.
	 */
	public static void condition(Supplier<Boolean> condition, long pollingRateMs) {
		while (!condition.get() && !Thread.interrupted()) {
			try {
				Thread.sleep(pollingRateMs);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				break;
			}
		}
	}

	/**
	 * Blocks the current thread until the given condition evaluates to true, polling at a fixed interval.
	 *
	 * @param condition   The condition to wait for.
	 * @param pollingRateMs The interval (in milliseconds) to check the condition.
	 * @param timeoutMs The maximum time to wait for the condition to be true, this is only checked when the condition is polled.
	 */
	public static void condition(Supplier<Boolean> condition, long pollingRateMs, double timeoutMs) {
		ElapsedTime timeoutTimer = new ElapsedTime();
		while (!condition.get() && !Thread.interrupted() && timeoutTimer.milliseconds() < timeoutMs) {
			try {
				Thread.sleep(pollingRateMs);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				break;
			}
		}
	}
}
