package org.firstinspires.ftc.teamcode.utilities;

import java.util.concurrent.*;
import java.util.function.Consumer;
import java.util.function.Function;

public class ChainedFuture<T> {
	private final CompletableFuture<T> future = new CompletableFuture<>();

	public void complete(T value) {
		future.complete(value);
	}

	public void completeExceptionally(Throwable ex) {
		future.completeExceptionally(ex);
	}

	public T get() throws InterruptedException, ExecutionException {
		return future.get();
	}

	public ChainedFuture<Void> thenRun(Consumer<T> action) {
		ChainedFuture<Void> nextFuture = new ChainedFuture<>();
		future.thenAccept(value -> {
			action.accept(value);
			nextFuture.complete(null);
		});
		return nextFuture;
	}

	public ChainedFuture<Void> thenRun(Runnable action) {
		ChainedFuture<Void> nextFuture = new ChainedFuture<>();
		future.thenRun(() -> {
			action.run();
			nextFuture.complete(null);
		});
		return nextFuture;
	}

	public <U> ChainedFuture<U> thenApply(Function<T, U> nextTask) {
		ChainedFuture<U> nextFuture = new ChainedFuture<>();
		future.thenApply(nextTask).thenAccept(nextFuture::complete);
		return nextFuture;
	}

	public boolean isDone(){
		return future.isDone();
	}
}
