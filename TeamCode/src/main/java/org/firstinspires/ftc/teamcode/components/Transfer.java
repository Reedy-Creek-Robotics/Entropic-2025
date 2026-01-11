package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

public class Transfer extends BaseComponent {

    /**
     * 0 = no balls in robot<br>
     * 1 = one ball in center<br>
     * 2 = one ball in center, one ball in front<br>
     * 3 = one ball in center, one ball in back<br>
     * 4 = three balls in robot
     */
    int ballState;

    private Servo rollerFront;
    private Servo rollerRear;

    LogCatUtil log;
    HardwareUtil hardwareUtil;

    Robot robot;

    public Transfer(RobotContext context, Robot robot) {
        super(context);

        log = new LogCatUtil("Transfer");
        this.robot = robot;

        hardwareUtil = new HardwareUtil(log, hardwareMap);
    }

    public Transfer(RobotContext context){
        this(context, null);
    }

    @Override
    public void init() {

        rollerFront = hardwareUtil.getServo("rollerFront");
        rollerRear = hardwareUtil.getServo("rollerRear");

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

    public void runFrontRoller(double power){
        rollerFront.setPosition((power + 1) / 2);
    }

    public void runRearRoller(double power){
        rollerRear.setPosition((power + 1) / 2);
    }

    public void rollerForTime(Servo roller, double power, double timeMs){
        robot.executeCommand(new RollerForTime(roller, power, timeMs));
    }

    public void rollersForTime(double power, double timeMs){
        robot.executeCommand(new RollersForTime(power, timeMs));
    }

/**BMS PLAN

first ball enters from either side<br>
  put in center position<br>
second ball enters from either side<br>
  first ball is moved to opposite side<br>
  second ball is moved to the center<br>
third ball enters<br>
  if on same side as second ball<br>
      put on side it entered in<br>
  otherwise<br>
      push first ball from center to opposite side of entry<br>
      push second ball from entry side to center<br>
      put on side it entered in<br>
**/
    public void incomingFront() {
        switch (ballState){
            case 0: //no ball in yet

                //run rollerFront in for 2 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 2000));
                break;

            case 1: //ball in center

                //run rollerBack out for 2 seconds
                executeCommand(new RollerForTime(rollerRear, -1, 2000));
                //run rollerFront in for 2 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 2000));
                break;

            case 2: //balls in center & front

                //run rollerFront in AND rollerBack out for 3 seconds
                executeCommand(new RollerForTime(rollerFront, 1, 3000));
                executeCommand(new RollerForTime(rollerRear, -1, 3000));
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
                executeCommand(new RollerForTime(rollerRear, 1, 2000));
                break;

            case 1: //ball in center

                //run rollerFront out for 2 seconds
                executeCommand(new RollerForTime(rollerFront, -1, 2000));
                //run rollerBack in for 2 seconds
                executeCommand(new RollerForTime(rollerRear, 1, 2000));
                break;

            case 2: //balls in center & front

                break;

            case 3: //balls in center & back

                break;
        }
    }

    private class RollerForTime implements Command {

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

    private class RollersForTime implements Command {

        double power, time;
        ElapsedTime timer;

        public RollersForTime(double power, double timeMs){
            this.power = power;
            this.time = timeMs;
        }

        @Override
        public void start(){
            runFrontRoller(power);
            runRearRoller(power);
            timer = new ElapsedTime();
        }

        @Override
        public void stop() {
            runFrontRoller(0);
            runRearRoller(0);
        }

        @Override
        public boolean update() {
            return timer.milliseconds() > time;
        }
    }
}
