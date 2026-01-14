package org.firstinspires.ftc.teamcode.opmodes;


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

    protected TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        panelsTelemetry.addLine("Waiting for start...");

        waitForStart();

        runPath();

        robot.saveStateToDisk();

    }

    public void initRobot(){
        robot = new Robot(this);
        robot.init();

        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    public void runPath(){

        robot.waitForCommandsToFinish();
    }




    /*public abstract void park();*/
}
