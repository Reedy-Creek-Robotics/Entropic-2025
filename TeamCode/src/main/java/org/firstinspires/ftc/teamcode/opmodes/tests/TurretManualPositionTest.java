package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
@Disabled
public class TurretManualPositionTest extends OpMode {

    static double ticksPerRev = 145.1;
    static double ticksPerDeg = ticksPerRev / 360;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    static double effectiveTicksPerRev = ticksPerRev * gearRatio;
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    DcMotorEx turret;
    Controller controller;
    double pos = 0;
    boolean simpleMovement = true;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        //turret.setPower(controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER) - controller.analogValue(Controller.AnalogControl.RIGHT_TRIGGER));
        telemetry.addData("power", turret.getPower());
        telemetry.addData("encoder ticks", turret.getCurrentPosition());
        telemetry.addData("velocity", turret.getVelocity());
        telemetry.addData("pos", pos);
        telemetry.addData("simple movement", simpleMovement);
        telemetry.addData("difference", turret.getCurrentPosition() - pos);

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            pos = effectiveTicksPerDeg * -0;
        }
        if(controller.isPressed(Controller.Button.DPAD_RIGHT)){
            pos = effectiveTicksPerDeg * -90;
        }
        if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            pos = effectiveTicksPerDeg * -180;
        }
        if(controller.isPressed(Controller.Button.DPAD_LEFT)){
            //pos = effectiveTicksPerDeg * 270;
        }

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            simpleMovement = !simpleMovement;
            turret.setMode(simpleMovement ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(simpleMovement) {
            if (Math.abs(turret.getCurrentPosition() - pos) <= 2) {
                turret.setPower(0);
            } else if (Math.abs(turret.getCurrentPosition() - pos) <= 30) {
                turret.setPower(turret.getCurrentPosition() < (int) pos ? 0.05 : -0.05);
            } else {
                turret.setPower(turret.getCurrentPosition() < (int) pos ? 0.2 : -0.2);
            }
        }else{
            turret.setTargetPosition((int) pos);
            turret.setPower(1);
        }

        telemetry.update();
    }
}
