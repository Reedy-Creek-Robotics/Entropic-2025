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
    private DualShooter shooter;

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
        super.update();
    }

    public boolean isTransferBusy(){
            return super.isBusy();
    }

    public void stopAllTransferCommands(){
        super.stopAllCommands();
    }

    public void setWaitForStateChange(boolean waitForStateChange){
        this.waitForStateChange = waitForStateChange;
    }

    @Override
    public void addTelemetry(){
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
        super.executeCommand(new RollerForTime(roller, power, timeMs));
    }

    public void rollersForTime(double power, double timeMs){
        super.executeCommand(new RollersForTime(power, timeMs));
    }


    public void swapSide(){
        switch (ballState){
            case 0:
            case 6:
                log.debug("swapSide | presuming incorrect ballState, setting to 3");
                ballState = 3;
                break;
            case 1:
                super.executeCommand(new RollersUntilSensor(1, -1, endoscope.getRearBallSensor(), 2, "swapSide bS=1 | front to rear"));
                break;
            case 2:
                super.executeCommand(new RollersUntilSensor(-1, 1, endoscope.getFrontBallSensor(), 1, "swapSide bS=2 | rear to front"));
                break;
            case 3:
                super.executeCommand(new RollersUntilSensor(0, -1, endoscope.getRearBallSensor(), -1, "swapSide bS=3 | 1/2 center to rear"));
                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(),4, "swapSide bS=3 | 2/2 front to center"));
                break;
            case 4:
                super.executeCommand(new RollersUntilSensor(-1, 0, endoscope.getFrontBallSensor(), -1, "swapSide bS=4 | 1/2 center to front"));
                super.executeCommand(new RollersUntilSensor(0, 1, endoscope.getCenterBallSensor(), 3, "swapSide bS=4 | 2/2 rear to center"));
                break;
            case 5:
                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4, "swapSide bS=5 | front to center"));
                break;
        }
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

                super.executeCommand(new RollersUntilSensor(0.7, -1, endoscope.getRearBallSensor(), 2, "incomingFront bS=0 | new to rear"));

                waitForStateChange = true;
                break;

            case 1: //one ball in front

                super.executeCommand(new RollersUntilSensor(0.7, -1, endoscope.getRearBallSensor(), -1, "incomingFront bS=1 | 1/2 front to rear"));
                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4, "incomingFront bS=1 | 2/2 new to center"));

                waitForStateChange = true;
                break;

            case 2: //one ball in rear

                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 4, "incomingFront bS=2 | new to center"));


                waitForStateChange = true;
                break;

            case 3: //two balls in front + center

                super.executeCommand(new RollersUntilSensor(0, -1, endoscope.getRearBallSensor(), -1, "incomingFront bS=3 | 1/2 center to rear"));
                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 6, "incomingFront bS=3 | 2/2 new to center"));

                waitForStateChange = true;
                break;

            case 4: //two balls in rear + center

                ballState = 6;

                //waitForStateChange = true;
                break;

            case 5: //two balls in front + rear

                super.executeCommand(new RollersUntilSensor(1, 0, endoscope.getCenterBallSensor(), 6, "incomingFront bS=5 | front to center"));


                waitForStateChange = true;
                break;

            case 6: //three balls in robot

                telemetry.addLine("Robot Full!");

                //waitForStateChange = true;
                break;
        }
    }
}

    public void incomingRear() {
        switch(ballState){
            case 0: //no balls in robot

                super.executeCommand(new RollersUntilSensor(-1,0.7, endoscope.getFrontBallSensor(), 1, "incomingRear bS=0 | new to front"));

                waitForStateChange = true;
                break;

            case 1: //one ball in front

                super.executeCommand(new RollersUntilSensor(0, 1, endoscope.getCenterBallSensor(), 3, "incomingRear bS=1 | new to center"));

                waitForStateChange = true;
                break;

            case 2: //one ball in rear

                super.executeCommand(new RollersUntilSensor(-1, 0.7, endoscope.getFrontBallSensor(), -1, "incomingRear bS=2 | 1/2 rear to front"));
                super.executeCommand(new RollersUntilSensor(0, 1, endoscope.getCenterBallSensor(),3, "\"incomingRear bS=2 | 2/2 new to center"));

                waitForStateChange = true;
                break;

            case 3: //two balls in front + center

                ballState = 6;

                //waitForStateChange = true;
                break;

            case 4: //two balls in rear + center

                super.executeCommand(new RollersUntilSensor(-1, 0, endoscope.getFrontBallSensor(), -1, "incomingRear bS=4 | center to front"));
                super.executeCommand(new RollersUntilSensor(0,1, endoscope.getCenterBallSensor(), 6, "\"incomingRear bS=4 | new to center"));

                waitForStateChange = true;
                break;

            case 5: //two balls in front + rear

                super.executeCommand(new RollersUntilSensor(0, 1, endoscope.getCenterBallSensor(), 6, "incomingRear bS=5 | rear to center"));

                waitForStateChange = true;
                break;

            case 6: //three balls in robot

                telemetry.addLine("Robot Full!");

                //waitForStateChange = true;
                break;
        }
    }

private class RollersUntilSensor implements Command {

    double powerFront, powerRear;
    String description;
    int finishState;
    PredominantColorProcessor sensor;
    ElapsedTime timeout;

    public RollersUntilSensor(double powerFront, double powerRear, PredominantColorProcessor sensor, int finishState){
        this.powerFront = powerFront;
        this.powerRear = powerRear;
        this.sensor = sensor;
        this.finishState = finishState;
        this.description = "unknown";
    }
    public RollersUntilSensor(double powerFront, double powerRear, PredominantColorProcessor sensor, int finishState, String description){
        this.powerFront = powerFront;
        this.powerRear = powerRear;
        this.sensor = sensor;
        this.finishState = finishState;
        this.description = description;
    }

    @Override
    public void start(){
        rollerFront.setPosition((powerFront + 1) / 2);
        rollerRear.setPosition((powerRear + 1) / 2);
        log.debug("STARTED (" + description + ") running front @ " + powerFront + "' and rear @ " + powerRear + " until sensor" + sensor.getName());
        timeout = new ElapsedTime();
    }

    @Override
    public void stop() {
        if (finishState != -1){
            ballState = finishState;
            waitForStateChange = false;
        }

        rollerRear.setPosition(0.5);
        rollerFront.setPosition(0.5);
        String msg = "STOPPED (" + description + ") running front @ " + powerFront + "' and rear @ " + powerRear + " until sensor" + sensor.getName() + "(took " + timeout.milliseconds() + "ms)";
        if (timeout.milliseconds() > 3000){
            log.warn("TIMEOUT!! " + msg);
        } else {
            log.debug(msg);
        }
    }

    @Override
    public boolean update() {
        telemetry.addLine("(" + description + ") moving front @ " + powerFront + " and rear @ " + powerRear + " until " + sensor.getName());
        return (endoscope.getPresence(sensor.getAnalysis().HSV) > 0) || (timeout.milliseconds() > 3000);
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
                case 4: // two balls in rear + center
                    ballState = 3; // two balls in front + center
                    break;
                case 3: // two balls in front + Center
                case 2: // one ball in rear
                    ballState = 1; // one ball in front
                    break;
                case 1: // one ball in front
                    ballState = 0; // no balls
                    break;
            }

            rollerFront.setPosition(0.5);
            rollerRear.setPosition(0.5);
        }

        @Override
        public boolean update() {
            telemetry.addLine("Serving Until Shot (" +  shooter.getCombinedCurrent() + "/" + shooter.shotCurrent + ")");
            return shooter.getCombinedCurrent() > shooter.shotCurrent;
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
