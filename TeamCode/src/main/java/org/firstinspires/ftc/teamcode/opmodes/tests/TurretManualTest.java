package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class TurretManualTest extends OpMode {

    static double ticksPerRev = 145.1;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    double effectiveTicksPerRev = 145.1 * gearRatio;


    DcMotorEx turret;
    Controller controller;
    double pos = 0;
    boolean test = true;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setTargetPositionTolerance(10);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        //turret.setPower(controller.analogValue(Controller.AnalogControl.LEFT_TRIGGER) - controller.analogValue(Controller.AnalogControl.RIGHT_TRIGGER));
        telemetry.addData("Power", turret.getPower());
        telemetry.addData("Encoder Ticks", turret.getCurrentPosition());
        telemetry.addData("Velocity", turret.getVelocity());
        telemetry.addData("Pos", pos);
        telemetry.addData("test", test);
        telemetry.addData("difference", turret.getCurrentPosition() - pos);
        if(controller.isPressed(Controller.Button.CROSS)){
            pos += effectiveTicksPerRev;
        }
        if(controller.isPressed(Controller.Button.CIRCLE)){
            pos -= effectiveTicksPerRev;
        }

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            test = !test;
            turret.setMode(test ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(test) {
            if (Math.abs(turret.getCurrentPosition() - pos) <= 3) {
                turret.setPower(0);
            } else if (Math.abs(turret.getCurrentPosition() - pos) <= 25) {
                turret.setPower(turret.getCurrentPosition() < (int) pos ? 0.2 : -0.2);
            } else {
                turret.setPower(turret.getCurrentPosition() < (int) pos ? 1 : -1);
            }
        }else{
            turret.setTargetPosition((int) pos);
            turret.setPower(1);
        }

        telemetry.update();
    }
}
