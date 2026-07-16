package org.firstinspires.ftc.teamcode.utilities;

import java.util.HashSet;
import java.util.Set;

public class Notifier {
	private boolean notified = false;
	private final Set<Thread> waitingThreads = new HashSet<>();

	/**
	 * Blocks the current thread until the notify method is called on the given Notifier.
	 */
	public void await() {
		synchronized (this) {
			resetNotification();
			Thread current = Thread.currentThread();
			waitingThreads.add(current);
			try {
				while (!notified && !Thread.interrupted()) {
					this.wait();
				}
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt(); // Preserve interrupt flag
			} finally {
				waitingThreads.remove(current);
			}
		}
	}

	/**
	 * Blocks the current thread until the notify method is called on this Notifier
	 * or until the specified timeout has elapsed. Please note the timeoutMs is not entirely accurate and may be off by 5-50 ms.
	 *
	 * @param timeoutMs The maximum time to wait in milliseconds before stopping.
	 */
	public void await(long timeoutMs) {
		synchronized (this) {
			resetNotification();
			Thread current = Thread.currentThread();
			waitingThreads.add(current);
			long endTime = System.currentTimeMillis() + timeoutMs;
			try {
				while (!notified && !Thread.interrupted()) {
					long remaining = endTime - System.currentTimeMillis();
					if (remaining <= 0) break;
					this.wait(remaining);
				}
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
			} finally {
				waitingThreads.remove(current);
			}
		}
	}

	public synchronized void notifyWaitingThreads() {
		this.notified = true;
		this.notifyAll();
	}

	public synchronized void interruptWaitingThreads() {
		for (Thread thread : waitingThreads) {
			thread.interrupt();
		}
		waitingThreads.clear();
	}

	public synchronized boolean isNotified() {
		return notified;
	}

	public synchronized void resetNotification() {
		this.notified = false;
	}
}