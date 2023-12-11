package org.firstinspires.ftc.teamcode.modules.statemachines;

import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;

public class LauncherSM implements StateMachine {

    STATE currentState = STATE.DRONE_LAUNCHER_LOADED;

    Launcher launcher;

    public enum STATE {
        DRONE_LAUNCHER_LOADED,
        DRONE_LAUNCHED,
        DRONE_LAUNCHER_RETRACTED,
    }

    public enum EVENT {
        GAME_START,
        DRONE_LAUNCH_BUTTON_PRESSED,
        DRONE_RETRACT_BUTTON_PRESSED,
    }

    public LauncherSM(Launcher launcher) {
        this.launcher = launcher;
    }

    public LauncherSM.STATE getState() {
        return currentState;
    }

    public void transition(EVENT event) {
        switch (event) {

        }
    }

    @Override
    public void update() {

    }
}
