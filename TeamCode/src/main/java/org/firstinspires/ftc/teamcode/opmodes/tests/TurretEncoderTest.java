package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TurretEncoderTest extends OpMode {
    DcMotorEx turret;
    TouchSensor resetSwitch;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        resetSwitch = hardwareMap.get(TouchSensor.class, "resetSwitch");
    }

    @Override
    public void loop() {
        telemetry.addData("pos", turret.getCurrentPosition());
        telemetry.addData("switch", resetSwitch.isPressed());
        telemetry.update();
    }
}
