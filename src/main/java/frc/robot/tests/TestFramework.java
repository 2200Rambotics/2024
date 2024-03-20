package frc.robot.tests;

import java.time.Duration;
import java.time.Instant;
import java.util.Vector;

public class TestFramework {
    enum TestOutputLevel {
        Benchmark,
        Normal,
        None
    }
    /** If true, tests are halted when one of them fails. */
    public static final boolean CANCEL_ON_FAILURE = true;
    /** If true, tests will print when they start. */
    public static final boolean NOTIFY_START = false;
    /** Defines the level of detail a test will report with. */
    public static final TestOutputLevel END_OUTPUT = TestOutputLevel.Benchmark;
    /**
     * If true, tests are allowed to be sent to their own threads.
     * Ensure that tests which can modify the global state are marked as synchronous by using `addSync`.
     */
    public static final boolean MULTITHREAD = false;

    /** The accumulated tests that were dispatched to separate threads. */
    static Vector<PureTest> pureTests = new Vector<>();
    /** The accumulated tests that are synchronous. */
    static Vector<SyncTest> syncTests = new Vector<>();
    public static void main(String... args) {
        new ExtraMathTests().addTests();

        for (SyncTest syncTest : syncTests) {
            runTest(syncTest.target, syncTest.name);
        }
        for (PureTest asyncTest : pureTests) {
            try { asyncTest.thread.join(); } catch (InterruptedException e) {
                System.out.println("! Test `" + asyncTest.name + "` was interrupted.");
            }
        }
    }
    /**
     * Schedules a test to be run.
     * 
     * By calling this function, you allow the test to be dispatched into a different thread. To cancel this behaviour, call `addSync` instead.
     * @param target The test to run.
     * @param name The name of the test.
     */
    public static void add(Runnable target, String name) {
        if (MULTITHREAD) {
            Thread thread = new Thread(()->runTest(target, name));
            thread.run();
            pureTests.add(new PureTest(thread, name));
        } else {
            addSync(target, name);
        }
    }
    /**
     * Schedules a test to be run. The test will not be placed in another thread.
     * @param target The test to run.
     * @param name The name of the test.
     */
    public static void addSync(Runnable target, String name) {
        syncTests.add(new SyncTest(target, name));
    }
    /**
     * The dispatched function that `add` calls or puts in a thread (depending on the configuration).
     * @param test The test to run.
     * @param name The name of the test.
     */
    static void runTest(Runnable test, String name) {
        if (NOTIFY_START) {
            System.out.println("Starting test `" + name + "`.");
        }
        Instant start = Instant.now();
        try {
            test.run();
        } catch (Throwable e) {
            System.err.println("!!! Test `" + name + "` failed!");
            if (CANCEL_ON_FAILURE) {
                System.err.println("Tests configured to halt on failure.");
                throw e;
            } else {
                System.err.println("Continuing with further tests, observe error below:");
                System.err.println(e);
            }
        }
        Instant end = Instant.now();
        switch (END_OUTPUT) {
            case Benchmark: System.out.println("Test `" + name + "` passed, took " + Duration.between(start, end) + '.'); break;
            case Normal: System.out.println("Test `" + name + "` passed."); break;
            case None: break;
        }
    }
}
