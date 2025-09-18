package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot;

public abstract class AutoMain extends LinearOpMode {
    protected Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        telemetry.log().add("Wait for start", "");

        waitForStart();

        runPath();

        robot.savePositionToDisk();

    }

    public void initRobot(){
        robot = new Robot(this);
        robot.init();

    }

    public void runPath(){

        robot.waitForCommandsToFinish();
    }




    /*public abstract void park();*/
}
