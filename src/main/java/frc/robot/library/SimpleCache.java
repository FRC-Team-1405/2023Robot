// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.library;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Add your docs here. */
public class SimpleCache<T> implements Supplier<T>, Consumer<T> {
    private Supplier<T> supplier;
    private Consumer<T> consumer;
    private T value;
    private long timeoutMilliseconds;
    private long randomStart=0;
    private long nextTime = 0;

    public SimpleCache(Supplier<T> supplier, Consumer<T> consumer, int timeoutMilliseconds) {
        this.supplier = supplier;
        this.consumer = consumer;
        this.timeoutMilliseconds = timeoutMilliseconds;
        randomStart = ThreadLocalRandom.current().nextInt(0, timeoutMilliseconds+1);
    }

    public T get(){
        long currentTime = System.currentTimeMillis();
        if (currentTime > nextTime) {
            value = supplier.get();
            if (nextTime == 0) {
                nextTime = currentTime + timeoutMilliseconds + randomStart;
            } else {
                nextTime = currentTime + timeoutMilliseconds;
            }
        }
        return value;
    }

    public void accept(T value){
        consumer.accept(value);
        this.value = value;
    }
}
