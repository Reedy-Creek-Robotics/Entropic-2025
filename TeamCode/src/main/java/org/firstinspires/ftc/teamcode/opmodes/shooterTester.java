package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class shooterTester extends OpMode {
Controller controller;
DcMotorEx shooter;
VoltageSensor voltage;

double power = 1500;
double avgInactiveCurrent = 1;

boolean reversed = true;
boolean active;

int velocityTolerance = 50;


    @Override
    public void init() {
        telemetry.speak("Test");
        telemetry.update();
        telemetry.clear();
        controller = new Controller(gamepad1);
        voltage = hardwareMap.voltageSensor.iterator().next();
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        telemetry.addLine("Controls:");
        telemetry.addLine("A to start");
        telemetry.addLine("LEFT STICK to reverse");
        telemetry.addLine("DPAD UP/DOWN to change power x100");
        telemetry.addLine("DPAD LEFT/RIGHT to change power x20");
        telemetry.addLine("------------------------");

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            power += 100;
        }else if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            power -= 100;
        }

        if(controller.isPressed(Controller.Button.DPAD_RIGHT)){
            power += 20;
        }else if(controller.isPressed(Controller.Button.DPAD_LEFT)){
            power -= 20;
        }

        if(power > 2240) power = 2240;
        if(power < 0) power = 0;

        if(controller.isPressed(Controller.Button.A)) {
            active = !active;
        }

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            reversed = !reversed;
        }

        telemetry.addData("Active", active);
        telemetry.addData("Reversed", reversed);
        telemetry.addData("Set Power (tps)", power);

        if(active){
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            shooter.setVelocity(power);
        }else{
            shooter.setPower(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /*
        ~1640 tps from 46in
        ~1600 tps from 77in
        ~1580 tps from 95in
        ~1540 tps from 63in
        ~1700 tps from 43in
        ~1720 tps from 123in
        ~1780 tps from 144in

        tps at 1 power: 2240
         */

        telemetry.addData("Velocity", shooter.getVelocity());
        telemetry.addData("Current", shooter.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Voltage", voltage.getVoltage());
        telemetry.addData("Avg Inactive", avgInactiveCurrent);

        if(shooter.getVelocity() > power - velocityTolerance && shooter.getVelocity() < power + velocityTolerance) {
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("SHOOT");

            if (shooter.getCurrent(CurrentUnit.AMPS) < avgInactiveCurrent + 0.5) {
                avgInactiveCurrent = (avgInactiveCurrent + shooter.getCurrent(CurrentUnit.AMPS)) / 2;
            }
        }
        telemetry.update();
    }
}
