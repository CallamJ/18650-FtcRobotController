package org.firstinspires.ftc.teamcode.utilities;

import java.util.concurrent.Callable;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class TaskScheduler {
	private static final TaskScheduler instance = new TaskScheduler(5);

	public static TaskScheduler getDefaultInstance(){
		return instance;
	}

	private final ScheduledExecutorService executor;
	private final Lock lock = new ReentrantLock();
	private final Condition condition = lock.newCondition();


	public TaskScheduler(int threadPoolSize) {
		this.executor = Executors.newScheduledThreadPool(threadPoolSize);
	}

	public <T> ChainedFuture<T> runAsync(Callable<T> task) {
		ChainedFuture<T> future = new ChainedFuture<>();
		executor.submit(() -> {
			try {
				future.complete(task.call());
			} catch (Exception e) {
				future.completeExceptionally(e);
			}
		});
		return future;
	}

	public <T> ChainedFuture<T> runAsync(Runnable task) {
		return runAsync(() -> {
			task.run();
			return null;
		});
	}

	public <T> ChainedFuture<T> runAsync(Runnable task, long delay, TimeUnit unit) {
		return runAsync(() -> {
			task.run();
			return null;
		}, delay, unit);
	}

	public <T> ChainedFuture<T> runAsync(Callable<T> task, long delay, TimeUnit unit) {
		ChainedFuture<T> future = new ChainedFuture<>();
		executor.schedule(() -> {
			try {
				future.complete(task.call());
			} catch (Exception e) {
				future.completeExceptionally(e);
			}
		}, delay, unit);
		return future;
	}

	public void shutdown() {
		executor.shutdown();
	}
}
