package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class EmptyObjectUtil {
    public static Object getEmpty(Object object){
        return new Object();
    }

    public static DcMotorEx getEmptyMotorEx(){
        // Creates a new DcMotor used to avoid null pointer errors, while not actually doing anything.
        return new DcMotorEx(){
            @Override
            public void setMotorEnable(){

            }

            @Override
            public void setMotorDisable(){

            }

            @Override
            public boolean isMotorEnabled(){
                return false;
            }

            @Override
            public void setVelocity(double angularRate){

            }

            @Override
            public void setVelocity(double angularRate, AngleUnit unit){

            }

            @Override
            public double getVelocity(){
                return 0;
            }

            @Override
            public double getVelocity(AngleUnit unit){
                return 0;
            }

            @Override
            public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients){

            }

            @Override
            public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

            }

            @Override
            public void setVelocityPIDFCoefficients(double p, double i, double d, double f){

            }

            @Override
            public void setPositionPIDFCoefficients(double p){

            }

            @Override
            public PIDCoefficients getPIDCoefficients(RunMode mode){
                return null;
            }

            @Override
            public PIDFCoefficients getPIDFCoefficients(RunMode mode){
                return null;
            }

            @Override
            public void setTargetPositionTolerance(int tolerance){

            }

            @Override
            public int getTargetPositionTolerance(){
                return 0;
            }

            @Override
            public double getCurrent(CurrentUnit unit){
                return 0;
            }

            @Override
            public double getCurrentAlert(CurrentUnit unit){
                return 0;
            }

            @Override
            public void setCurrentAlert(double current, CurrentUnit unit){

            }

            @Override
            public boolean isOverCurrent(){
                return false;
            }

            @Override
            public MotorConfigurationType getMotorType(){
                return null;
            }

            @Override
            public void setMotorType(MotorConfigurationType motorType){

            }

            @Override
            public DcMotorController getController(){
                return null;
            }

            @Override
            public int getPortNumber(){
                return 0;
            }

            @Override
            public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior){

            }

            @Override
            public ZeroPowerBehavior getZeroPowerBehavior(){
                return null;
            }

            @Override
            public void setPowerFloat(){

            }

            @Override
            public boolean getPowerFloat(){
                return false;
            }

            @Override
            public void setTargetPosition(int position){

            }

            @Override
            public int getTargetPosition(){
                return 0;
            }

            @Override
            public boolean isBusy(){
                return false;
            }

            @Override
            public int getCurrentPosition(){
                return 0;
            }

            @Override
            public void setMode(RunMode mode){

            }

            @Override
            public RunMode getMode(){
                return null;
            }

            @Override
            public void setDirection(Direction direction){

            }

            @Override
            public Direction getDirection(){
                return null;
            }

            @Override
            public void setPower(double power){

            }

            @Override
            public double getPower(){
                return 0;
            }

            @Override
            public Manufacturer getManufacturer(){
                return null;
            }

            @Override
            public String getDeviceName(){
                return "";
            }

            @Override
            public String getConnectionInfo(){
                return "";
            }

            @Override
            public int getVersion(){
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode(){

            }

            @Override
            public void close(){

            }
        };
    }

    public static Servo getEmptyServo(){
        // Creates a new Servo used to avoid null pointer errors, while not actually doing anything
        return new Servo(){
            @Override
            public Manufacturer getManufacturer(){
                return null;
            }

            @Override
            public String getDeviceName(){
                return "";
            }

            @Override
            public String getConnectionInfo(){
                return "";
            }

            @Override
            public int getVersion(){
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode(){

            }

            @Override
            public void close(){

            }

            @Override
            public ServoController getController(){
                return null;
            }

            @Override
            public int getPortNumber(){
                return 0;
            }

            @Override
            public void setDirection(Direction direction){

            }

            @Override
            public Direction getDirection(){
                return null;
            }

            @Override
            public void setPosition(double position){

            }

            @Override
            public double getPosition(){
                return 0;
            }

            @Override
            public void scaleRange(double min, double max){

            }
        };
    }

    public static WebcamName getEmptyWebcamName(){
        return new WebcamName(){
            @NonNull
            @Override
            public SerialNumber getSerialNumber(){
                return getEmptySerialNumber();
            }

            @Override
            public String getUsbDeviceNameIfAttached(){
                return "";
            }

            @Override
            public boolean isAttached(){
                return false;
            }

            @Override
            public Manufacturer getManufacturer(){
                return null;
            }

            @Override
            public String getDeviceName(){
                return "";
            }

            @Override
            public String getConnectionInfo(){
                return "";
            }

            @Override
            public int getVersion(){
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode(){

            }

            @Override
            public void close(){

            }

            @Override
            public boolean isWebcam(){
                return false;
            }

            @Override
            public boolean isCameraDirection(){
                return false;
            }

            @Override
            public boolean isSwitchable(){
                return false;
            }

            @Override
            public boolean isUnknown(){
                return false;
            }

            @Override
            public void asyncRequestCameraPermission(Context context, Deadline deadline, Continuation<? extends Consumer<Boolean>> continuation){

            }

            @Override
            public boolean requestCameraPermission(Deadline deadline){
                return false;
            }

            @Override
            public CameraCharacteristics getCameraCharacteristics(){
                return null;
            }
        };
    }

    public static SerialNumber getEmptySerialNumber(){
        return new SerialNumber("") {
            @Override
            public int hashCode() {
                return super.hashCode();
            }
        };
    }
}
