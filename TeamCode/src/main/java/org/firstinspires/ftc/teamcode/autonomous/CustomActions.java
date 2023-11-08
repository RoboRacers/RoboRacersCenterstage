package org.firstinspires.ftc.teamcode.autonomous;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

public class CustomActions {

    public static class ObserverThread extends Thread {

        public boolean interrupted = false;

        public Observer observer;

        public ObserverThread(Observer observer) {
            this.observer = observer;
        }

        public void run() {
            try {
                while (!this.isInterrupted()) {
                    interrupted = observer.checkForInterrupt();
                    sleep(10);
                }
            } catch (Exception e) {
                System.out.println("Exception caught: " + e);
            }
        }
    }

    public static class ActionThread extends Thread {

        public boolean finished = false;

        public Action action;

        public ActionThread(Action action) {
            this.action = action;
        }

        public void run() {
            try {
                boolean run = true;
                while (run && !this.isInterrupted()) {
                    Actions.runBlocking();
                    run = action.run(new TelemetryPacket());
                }
                finished = true;
            } catch (Exception e) {
                System.out.println("Exception caught: " + e);
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
        ObserverThread observerThread = new ObserverThread(observer);
        ActionThread actionThread = new ActionThread(action);

        observerThread.start();
        actionThread.start();
        while (!Thread.currentThread().isInterrupted()) {
            if (observerThread.interrupted) {
                // If the observer detects an interrupt, stop the current action and exit the loop
                actionThread.interrupt();
                observerThread.interrupt();
                break;
            } else if (actionThread.finished) {
                // If the action thread is finished, end both threads
                actionThread.interrupt();
                observerThread.interrupt();
                break;
            }
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
