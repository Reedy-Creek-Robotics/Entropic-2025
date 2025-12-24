package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareUtil {
    LogCatUtil log;
    HardwareMap map;

    public HardwareUtil(LogCatUtil log, HardwareMap map){
        this.log = log;
        this.map = map;
    }

    public DcMotorEx getMotorEx(String deviceName) {
        DcMotorEx result;
        try{
            result = map.get(DcMotorEx.class, deviceName);
        }catch(Exception e){
            log.hardwareCatch(deviceName, e);
            result = EmptyObjectUtil.getEmptyMotorEx();
        }
        
        return result;
    }

    public Servo getServo(String deviceName) {
        Servo result;
        try{
            result = map.get(Servo.class, deviceName);
        }catch(Exception e){
            log.hardwareCatch(deviceName, e);
            result = EmptyObjectUtil.getEmptyServo();
        }

        return result;
    }

    public SparkFunOTOS getOtos(String deviceName) {
        SparkFunOTOS result;
        try{
            result = map.get(SparkFunOTOS.class, deviceName);
        }catch(Exception e){
            log.hardwareCatch(deviceName, e);
            result = EmptyObjectUtil.getEmptySparkFunOTOS();
        }

        return result;
    }

    public WebcamName getWebcamName(String deviceName) {
        WebcamName result;
        try{
            result = map.get(WebcamName.class, deviceName);
        }catch(Exception e){
            log.hardwareCatch(deviceName, e);
            result = EmptyObjectUtil.getEmptyWebcamName();
        }

        return result;
    }
}
