package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.log.Datalog;
import java.text.SimpleDateFormat;

@TeleOp(name = "Logging: Bionic", group = "Logging")
public class ShooterTesterAdvancedBionic extends LinearOpMode {
    double powerIncrement = 0.05;

    Datalog log;

    /***
     * dpad = up and down to increment/decrement motor power by 0.2
     * dpad = side to side cycle through motors
     *   - create a static map of motors: 117, 312, 435, 1150
     *   - map also includes number of ticks to use for encoder movements
     * X = start the program
     * Y = stop
     * log file = every loop, comma separated
     *  - timestamp
     *  - current motor current
     *  - encoder ticks since start
     *  - log start and stop button pressed
     *  - should use a unique filename that includes timestamp
     *  - create the file if it does not exist during init
     *  - close the file on stop
     *  output to the screen current motor current, encoder ticks since start
     */
    @Override
    public void runOpMode() throws InterruptedException {
        //serveFiles();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // current power setting
        double motorPower = 0.0;

        // init the file logging
        // add a timestamp on end of filename so each run of op mode gives
        // you a unique file
        String timeStamp = new SimpleDateFormat("yyyyMMdd-HHmmss").format(new java.util.Date());
        log = new Datalog("shooterTester_" + timeStamp);

        // WAIT FOR INIT
        waitForStart();

        boolean isRunning = false;
        boolean loggingEnabled = false;
        while(opModeIsActive()) {
            // controls
            if( gamepad1.dpadDownWasPressed() ) {
                motorPower -= powerIncrement;
            }

            if( gamepad1.dpadUpWasPressed() ) {
                motorPower += powerIncrement;
            }

            // start the test
            if( gamepad1.squareWasPressed() ) {
                isRunning = true;
                motor.setPower(motorPower);
            }

            // stop the test
            if( gamepad1.triangleWasPressed() ) {
                isRunning = false;
                motor.setPower(0);
            }

            if( gamepad1.circleWasPressed()) {
                loggingEnabled = !loggingEnabled;
            }

            // collect data
            double actualPower = motor.getPower();
            double current = motor.getCurrent(CurrentUnit.AMPS);
            double velocity = motor.getVelocity();
            double ticks = motor.getCurrentPosition();

            // log - only when logging is enabled
            if( loggingEnabled ) {
                log.current.set(current);
                log.running.set(isRunning);
                log.ticks.set(ticks);
                log.motorPower.set(motorPower);
                log.velocity.set(velocity);
                log.batteryVoltage.set(getBatteryVoltage());
                log.writeLine();
            }

            // telemetry
            telemetry.addData("set power", motorPower);
            telemetry.addData("actual power", motor.getPower());
            telemetry.addData("current", current);
            telemetry.addData("ticks", ticks );
            telemetry.addData("velocity",velocity);
            telemetry.update();
        }

        // Clean shutdown
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


}