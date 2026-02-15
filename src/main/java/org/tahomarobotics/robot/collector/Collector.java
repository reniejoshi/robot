package org.tahomarobotics.robot.collector;

public class Collector {
    private final CollectorSubsystem collector;

    public Collector() {
        this(new CollectorSubsystem());
    }

    Collector(CollectorSubsystem collector) {
        this.collector = collector;
    }
}
