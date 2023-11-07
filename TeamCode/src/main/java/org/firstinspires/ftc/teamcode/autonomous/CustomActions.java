package org.firstinspires.ftc.teamcode.autonomous;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

public class CustomActions {

    public static class ObserverThread extends Thread {

        public boolean interrupted = false;

        public Observer observer;

        public ObserverThread(Observer observer) {
            this.observer = observer;
        }

        public void run() {
            try {
                interrupted = observer.checkForInterrupt();
            } catch (Exception e) {

            }
        }
    }

    public static class ActionThread extends Thread {

        public boolean repeat;

        public Action action;

        public ActionThread(Action action) {
            this.action = action;
        }

        public void run() {
            try {
                TelemetryPacket t = new TelemetryPacket();
                repeat = action.run(t);
            } catch (Exception e) {

            }
        }
    }

    /**
     * Observer class that checks for interruptions.
     * Used in this version of .runBlocking
     */
    public interface Observer {
        /**
         * Returns true if no interrupt, returns false if interrupt.
         * @return Interrupt
         */
        boolean checkForInterrupt();
    }

    /**
     * runBlocking as seen in Vanilla Roadrunner 1.0, but checks for interruptions
     * @param action
     * @param observer
     */
    public static void runBlocking(Action action, Observer observer) {
        boolean b = true;
        ObserverThread observerThread = new ObserverThread(observer);
        ActionThread actionThread = new ActionThread(action);

        observerThread.run();
        while (b && !Thread.currentThread().isInterrupted()) {

            TelemetryPacket p = new TelemetryPacket();
            actionThread.run();

            b =
        }
    }

    public static void main(String[] args) {

        boolean someCondition = false;

        // Example of running this version of .runBlocking();
        CustomActions.runBlocking(
                new SequentialAction(
                        (telemetryPacket) -> {
                            System.out.println("Some Action");
                            return false;
                        },
                        (telemetryPacket) -> {
                            System.out.println("Some other Action");
                            return false;
                        }
                ),
                () -> { // The Observer
                    if (someCondition) {
                        return false;
                    } else {
                        return true;
                    }

                }
        );
    }

}
