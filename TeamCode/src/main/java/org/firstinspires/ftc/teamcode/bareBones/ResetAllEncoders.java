package org.firstinspires.ftc.teamcode.bareBones;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

@TeleOp(name = "Reset Encoders", group = "Extras")
public class ResetAllEncoders extends LinearOpMode {

    List<DcMotorEx> motors;
    List<String> motorNames;

    @Override
    public void runOpMode() throws InterruptedException {
        motors = hardwareMap.getAll(DcMotorEx.class);
        motorNames = new ArrayList<>();
        for(DcMotorEx motor : motors) {
            motorNames.add(hardwareMap.getNamesOf(motor).iterator().next());
        }

        telemetry.setAutoClear(true);

        if(motors.isEmpty()) throw new InterruptedException("No motors found in HardwareMap");

        telemetry.addLine("Start OpMode to reset all encoders");
        telemetry.addData("Motors Found", motors.size());
        telemetry.addLine();
        telemetry.update();

        waitForStart();

        for(DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while(timer.time() < 15.5 && opModeIsActive()){
            telemetry.addData("Motors Reset", motorNames.toString());
            telemetry.addLine();
            telemetry.addLine("---------------------------------------");
            telemetry.addLine();
            telemetry.addLine("Done!");
            telemetry.addData("Exiting in", Math.max(15 - ((int) timer.time()), 0));
            telemetry.update();
        }
    }
}
