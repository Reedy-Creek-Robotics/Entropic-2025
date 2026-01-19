package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.teamcode.custom.PredominantColorProcessor;

import java.util.Arrays;

public class UpgradedTransfer extends BaseComponent {

    /**
     * 0 = no balls in robot<br>
     * 1 = one ball in front<br>
     * 2 = one ball in rear<br>
     * 3 = two balls in front + center<br>
     * 4 = two balls in rear + center<br>
     * 5 = two balls in front + rear<br>
     * 6 = three balls in robot
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

    public UpgradedTransfer(RobotContext context, Robot robot) {
        super(context);

        log = new LogCatUtil("Transfer");
        this.robot = robot;

        hardwareUtil = new HardwareUtil(log, hardwareMap);
    }

    public UpgradedTransfer(RobotContext context){
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



    public void rollerForTime(Servo roller, double power, double timeMs){
        robot.executeCommand(new RollerForTime(roller, power, timeMs));
    }

    public void rollersForTime(double power, double timeMs){
        robot.executeCommand(new RollersForTime(power, timeMs));
    }



/**BMS PLAN
 first ball enters from either side
 push to opposite side

 second ball enters same side
 push to center

 second ball enters opposite side
 push first ball to opposite side
 push second ball to center

 third ball enters same as second
 leave on same side

 third ball enters opposite second
 push everything back over
 leave on opposite side

 third ball enters with balls front and rear
 push ball already on same side to center
 leave on whichever side

**/
public void incomingFront() {
    telemetry.addData("front income detected. ballState", ballState);
    if(!waitForStateChange) {
        switch (ballState) {
            case 0: //no balls in robot

                robot.executeCommand(new RollersUntilSensor(0.7, -1, endoscope.getRearBallSensor(), 2));

                waitForStateChange = true;
                break;

            case 1: //one ball in front

                robot.executeCommand(new RollersUntilSensor(0.7, -1, endoscope.getRearBallSensor(), -1));
                robot.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4));

                waitForStateChange = true;
                break;

            case 2: //one ball in rear

                robot.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4));


                waitForStateChange = true;
                break;

            case 3: //two balls in front + center

                robot.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4));


                waitForStateChange = true;
                break;

            case 4: //two balls in rear + center

                waitForStateChange = true;
                break;

            case 5: //two balls in front + rear


                waitForStateChange = true;
                break;

            case 6: //three balls in robot

                waitForStateChange = true;
                break;
        }
    }
}

    public void incomingRear() {
        switch(ballState){
            case 0: //no balls in robot

                waitForStateChange = true;
                break;

            case 1: //one ball in front

                waitForStateChange = true;
                break;

            case 2: //one ball in rear

                waitForStateChange = true;
                break;

            case 3: //two balls in front + center

                waitForStateChange = true;
                break;

            case 4: //two balls in rear + center

                waitForStateChange = true;
                break;

            case 5: //two balls in front + rear


                waitForStateChange = true;
                break;

            case 6: //three balls in robot

                waitForStateChange = true;
                break;
        }
    }

private class RollersUntilSensor implements Command {

    double powerFront, powerRear;
    int finishState;
    PredominantColorProcessor sensor;

    public RollersUntilSensor(double powerFront, double powerRear, PredominantColorProcessor sensor, int finishState){
        this.powerFront = powerFront;
        this.powerRear = powerRear;
        this.sensor = sensor;
        this.finishState = finishState;
    }

    @Override
    public void start(){
        rollerFront.setPosition((powerFront + 1) / 2);
        rollerRear.setPosition((powerRear + 1) / 2);
    }

    @Override
    public void stop() {
        if (finishState != -1){
            ballState = finishState;
            waitForStateChange = false;
        }

        rollerRear.setPosition(0.5);
        rollerFront.setPosition(0.5);
    }

    @Override
    public boolean update() {
        telemetry.addLine("moving front @ '" + powerFront + "' and rear @ '" + powerRear + "' until sensor" + sensor.getName());
        return endoscope.getPresence(sensor.getAnalysis().HSV) > 0;
    }
}

    private class ServeUntilShot implements Command {

        public ServeUntilShot() {

        }

        @Override
        public void start(){
            rollerFront.setPosition(1);
            rollerRear.setPosition(1);
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

            rollerFront.setPosition(0.5);
            rollerRear.setPosition(0.5);
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
