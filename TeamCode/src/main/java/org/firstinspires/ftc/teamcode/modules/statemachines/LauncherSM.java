package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;

public class LauncherSM implements StateMachine {

    STATE currentState;

    Launcher launcher;

    public enum STATE {
        DRONE_LAUNCHER_LOADED,
        DRONE_LAUNCHED,
    }

    public enum EVENT {
        GAME_START,
        DRONE_LAUNCH_BUTTON_PRESSED,
    }

    public LauncherSM(Launcher launcher) {
        this.launcher = launcher;
    }

    public LauncherSM.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {
            case GAME_START:
                currentState = LauncherSM.STATE.DRONE_LAUNCHER_LOADED;
                break;
            case DRONE_LAUNCH_BUTTON_PRESSED:
                currentState = LauncherSM.STATE.DRONE_LAUNCHED;
                break;
        }
    }

    @Override
    public void update() {
        switch (currentState) {
            case DRONE_LAUNCHED:
                launcher.actuationServo.setPower(500); // not done and not right thing;
                break;
            case DRONE_LAUNCHER_LOADED:
                break;
        }
    }
}
