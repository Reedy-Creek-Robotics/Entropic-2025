package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.EmptyObjectUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

public class Transtake extends BaseComponent {

    /**
     * 0 = no balls in robot<br>
     * 1 = one ball in center<br>
     * 2 = one ball in center, one ball in front<br>
     * 3 = one ball in center, one ball in back<br>
     * 4 = three balls in robot
     */
    int ballState;

    private DcMotorEx intakeFront;
    private DcMotorEx intakeBack;
    private Servo rollerFront;
    private Servo rollerBack;

    LogCatUtil log;

    public Transtake(RobotContext context) {
        super(context);

        log = new LogCatUtil("Transtake");
    }

    @Override
    public void init() {
        try {
            intakeFront = hardwareMap.get(DcMotorEx.class, "intakeFront");
            intakeBack = hardwareMap.get(DcMotorEx.class, "intakeBack");
        }catch (Exception e){
            log.error("\"intakeFront\" or \"intakeBack\" not found in hardware map. Falling back to empty DcMotorEx object.");
            log.error(e.getMessage());
            intakeFront = EmptyObjectUtil.getEmptyMotorEx();
            intakeBack = EmptyObjectUtil.getEmptyMotorEx();
        }
        try {
            rollerFront = hardwareMap.get(Servo.class, "rollerFront");
            rollerBack = hardwareMap.get(Servo.class, "rollerBack");
        } catch (Exception e) {
            log.error("\"rollerFront\" or \"rollerBack\" not found in hardware map. Falling back to empty DcMotorEx object.");
            log.error(e.getMessage());
            rollerFront = EmptyObjectUtil.getEmptyServo();
            rollerBack = EmptyObjectUtil.getEmptyServo();
        }

        //change depending on auto - may need to grab from file
        ballState = 0;
    }

    @Override
    public void update(){
        telemetry.addData("ballState", ballState);
    }

    public int getBallState() {
        return ballState;
    }

    public void setBallState(int ballState) {
        this.ballState = ballState;
    }

//BMS PLAN

//first ball enters from either side
//  put in center position
//second ball enters from either side
//  first ball is moved to opposite side
//  second ball is moved to the center
//third ball enters
//  if on same side as second ball
//      put on side it entered in
//  otherwise
//      push first ball from center to opposite side of entry
//      push second ball from entry side to center
//      put on side it entered in

    public void incomingFront() {
        switch (ballState){
            case 0: //no ball in yet

                //run rollerFront in for 2 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 2000));
                break;

            case 1: //ball in center

                //run rollerBack out for 2 seconds
                executeCommand(new RollerForTime(rollerBack, -1, 2000));
                //run rollerFront in for 2 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 2000));
                break;

            case 2: //balls in center & front

                //run rollerFront in AND rollerBack out for 3 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 3000));
                executeCommand(new RollerForTime(rollerBack, -1, 3000));
                break;

            case 3: //balls in center & back

                //run rollerFront in for 2 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 2000));
                break;
        }
    }

    public void incomingBack() {
        switch(ballState){
            case 0: //no ball in yet

                //run rollerBack in for 2 seconds
                executeCommand(new RollerForTime(rollerBack, 1, 2000));
                break;

            case 1: //ball in center

                //run rollerFront out for 2 seconds
                executeCommand(new RollerForTime(rollerFront, -1, 2000));
                //run rollerBack in for 2 seconds
                executeCommand(new RollerForTime(rollerBack, 1, 2000));
                break;

            case 2: //balls in center & front

                break;

            case 3: //balls in center & back

                break;
        }
    }

    public class RollerForTime implements Command {

        Servo roller;
        double power, time;
        ElapsedTime timer;

        public RollerForTime(Servo roller, double power, double timeMs){
            this.roller = roller;
            this.power = power;
            this.time = timeMs;
        }

        @Override
        public void start(){
            roller.setPosition((power + 1) / 2);
            timer = new ElapsedTime();
        }

        @Override
        public void stop() {
            roller.setPosition(0.5);
        }

        @Override
        public boolean update() {
            return timer.milliseconds() > time;
        }
    }
}
