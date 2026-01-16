package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

public abstract class AutoMain extends LinearOpMode {
    protected Robot robot;

    protected LogCatUtil log;

    protected Follower follower;
    protected Timer pathTimer, opmodeTimer;

    protected int pathState;
    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    protected boolean alliance;
    protected boolean running = true;

    protected TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        buildPaths();
        initAuto();
        panelsTelemetry.addLine("Waiting for start...");

        waitForStart();
        while(running && opModeIsActive()) {
            runPath();
            robot.update();
        }

        robot.saveStateToDisk();
    }

    public void initRobot(){
        robot = new Robot(this, alliance);
        robot.init();

        follower = robot.getDriveTrain().getFollower();
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    /**
     * Run once on init
     */
    public abstract void buildPaths();

    public abstract void initAuto();

    /**
     * Looped every iteration until <b>running</b> is false
     */
    public void runPath(){
        robot.waitForCommandsToFinish();
    }


    /*public abstract void park();*/
}
