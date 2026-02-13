package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class BeamBreakTest extends OpMode {
    TouchSensor beamBreak;

    @Override
    public void init() {
        beamBreak = hardwareMap.get(TouchSensor.class, "shootSwitch");
    }

    @Override
    public void loop() {
        telemetry.addData("Value from sensor isPressed", beamBreak.isPressed());
        telemetry.addData("Value from sensor getValue", beamBreak.getValue());
        telemetry.update();
    }
}
