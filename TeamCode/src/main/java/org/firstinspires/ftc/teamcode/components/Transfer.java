package org.firstinspires.ftc.teamcode.components;

import android.app.Presentation;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Arrays;

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
    private Endoscope endoscope;
    private Shooter shooter;

    LogCatUtil log;
    HardwareUtil hardwareUtil;

    Robot robot;
    Command transferCommand;
    Boolean waitForStateChange = false;

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
        endoscope = robot.getEndoscope();
        shooter = robot.getShooter();
        transferCommand = null;

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

    // power > 0 means ball is pushed in and up
    // power < 0 means ball is pushed down and out
    public void runFrontRoller(double power){
        rollerFront.setPosition((power + 1) / 2);
    }

    public void runRearRoller(double power){
        rollerRear.setPosition((power + 1) / 2);
    }


    public void serveUntilShot(int color){
        robot.executeCommand(new ServeUntilShot(color));
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
    telemetry.addData("front income detected. ballState", ballState);
    if(!waitForStateChange) {
        switch (ballState) {
            case 0: //no ball in yet

                //run rollerFront in
                robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getCenterBallSensor(), 1, 1));

                waitForStateChange = true;
                break;

            case 1: //ball in center

                //run rollerRear out
                robot.executeCommand(new RollerUntilSensor(rollerRear, endoscope.getRearBallSensor(), -1, -1));
                //run rollerFront in
                robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getCenterBallSensor(), 1, 3));
                waitForStateChange = true;
                break;

            case 2: //balls in center & front

                //run rollerFront in AND rollerRear out
                robot.executeCommand(new RollerUntilSensor(rollerRear, endoscope.getRearBallSensor(), -1, -1));
                robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getCenterBallSensor(), 1, 4));
                waitForStateChange = true;
                break;

            case 3: //balls in center & rear

                //run rollerFront in
                //robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getFrontBallSensor(), 1, 4));
                ballState = 4;
                break;
        }
    }
}

    public void incomingRear() {
        switch(ballState){
            case 0: //no ball in yet

                //run rollerRear
                robot.executeCommand(new RollerUntilSensor(rollerRear, endoscope.getCenterBallSensor(), 1, 1));
                waitForStateChange = true;
                break;

            case 1: //ball in center

                //run rollerFront out
                robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getFrontBallSensor(), -1, -1));
                //run rollerBack
                robot.executeCommand(new RollerUntilSensor(rollerRear, endoscope.getCenterBallSensor(), 1, 2));
                waitForStateChange = true;
                break;

            case 2: //balls in center & front
                //robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getCenterBallSensor(), 1, 1));
                ballState = 4;
                break;

            case 3: //balls in center & back
                //run rollerFront out
                //run rollerRear in
                robot.executeCommand(new RollerUntilSensor(rollerFront, endoscope.getFrontBallSensor(), -1, -1));
                robot.executeCommand(new RollerUntilSensor(rollerRear, endoscope.getCenterBallSensor(), 1, 4));
                waitForStateChange = true;
                break;
        }
    }

private class RollerUntilSensor implements Command {

    Servo roller;
    double power;
    int finishState;
    PredominantColorProcessor sensor;

    public RollerUntilSensor(Servo roller, PredominantColorProcessor sensor, double power, int finishState){
        this.roller = roller;
        this.power = power;
        this.sensor = sensor;
        this.finishState = finishState;
    }

    @Override
    public void start(){
        roller.setPosition((power + 1) / 2);
    }

    @Override
    public void stop() {
        if (finishState != -1){
            ballState = finishState;
            waitForStateChange = false;
        }

        rollerForTime(roller, power, 50);
    }

    @Override
    public boolean update() {
        roller.setPosition((power + 1) / 2);
        telemetry.addLine("moving roller '" + roller.getPortNumber() + "' until sensor" + Arrays.toString(sensor.getAnalysis().HSV));
        return endoscope.getPresence(sensor.getAnalysis().HSV) > 0;
    }
}

    private class ServeUntilShot implements Command {

        /**
        Colors:
         0 = either one
         1 = purple
         2 = green
         **/
        int color;

        public ServeUntilShot(int color) {
            this.color = color;
        }

        @Override
        public void start(){
            runFrontRoller(1);
            runRearRoller(1);
        }

        @Override
        public void stop() {
            switch (ballState){
                case 4:
                    ballState = 3;
                    break;
                case 3:
                case 2:
                    ballState = 1;
                    break;
                case 1:
                    ballState = 0;
                    break;
            }

            runFrontRoller(0);
            runRearRoller(0);
        }

        @Override
        public boolean update() {
            telemetry.addLine("Serving Until Shot (" +  shooter.getShooterCurrent() + "/" + shooter.shotCurrent + ")");
            return shooter.getShooterCurrent() > shooter.shotCurrent;
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
