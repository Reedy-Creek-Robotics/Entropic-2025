package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.graphics.ImageFormat;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.jetbrains.annotations.Contract;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class EmptyObjectUtil {
    public static HardwareDevice getEmpty(String name){
        if(name.equals(DcMotorEx.class.getSimpleName())){
            return getEmptyMotorEx();
        } else if(name.equals(Servo.class.getSimpleName())){
            return getEmptyServo();
        } else if(name.equals(WebcamName.class.getSimpleName())){
            return getEmptyWebcamName();
        } else if(name.equals(SparkFunOTOS.class.getSimpleName())){
            return getEmptySparkFunOTOS();
        } else{
            return getEmptyHardwareDevice();
        }
    }

    @NonNull
    @Contract(" -> new")
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

    @NonNull
    @Contract(" -> new")
    public static TouchSensor getEmptyTouchSensor(){
        return new TouchSensor() {
            @Override
            public double getValue() {
                return 0;
            }

            @Override
            public boolean isPressed() {
                return false;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    @NonNull
    @Contract(" -> new")
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

    @NonNull
    @Contract(" -> new")
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
                return true;
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
                return getEmptyCameraCharacteristics();
            }
        };
    }

    @NonNull
    @Contract(" -> new")
    public static SerialNumber getEmptySerialNumber(){
        return new SerialNumber("") {
            @Override
            public int hashCode() {
                return super.hashCode();
            }
        };
    }

    @NonNull
    @Contract(" -> new")
    public static HardwareDevice getEmptyHardwareDevice(){
        return new HardwareDevice() {
            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }
        };
    }

    @NonNull
    @Contract(" -> new")
    public static CameraCharacteristics getEmptyCameraCharacteristics(){
        return new CameraCharacteristics() {
            @Override
            public int[] getAndroidFormats() {
                return new int[0];
            }

            @Override
            public Size[] getSizes(int androidFormat) {
                return new Size[]{new Size(640, 480)};
            }

            @Override
            public Size getDefaultSize(int androidFormat) {
                return new Size(640, 480);
            }

            @Override
            public long getMinFrameDuration(int androidFormat, Size size) {
                return 100;
            }

            @Override
            public int getMaxFramesPerSecond(int androidFormat, Size size) {
                return 30;
            }

            @Override
            public List<CameraMode> getAllCameraModes() {
                return Collections.singletonList(new CameraMode(ImageFormat.JPEG, new Size(640, 480), 100, true));
            }
        };
    }

    @NonNull
    @Contract(" -> new")
    public static SparkFunOTOS getEmptySparkFunOTOS(){
        return new EmptySparkFunOtos();
    }

    @NonNull
    @Contract(" -> new")
    public static I2cDeviceSynch getEmptyI2cDeviceSynch(){
        return new I2cDeviceSynch() {
            @Override
            public void setReadWindow(ReadWindow window) {

            }

            @Override
            public ReadWindow getReadWindow() {
                return null;
            }

            @Override
            public void ensureReadWindow(ReadWindow windowNeeded, ReadWindow windowToSet) {

            }

            @Override
            public TimestampedData readTimeStamped(int ireg, int creg, ReadWindow readWindowNeeded, ReadWindow readWindowSet) {
                return null;
            }

            @Override
            public void setHeartbeatInterval(int ms) {

            }

            @Override
            public int getHeartbeatInterval() {
                return 0;
            }

            @Override
            public void setHeartbeatAction(HeartbeatAction action) {

            }

            @Override
            public HeartbeatAction getHeartbeatAction() {
                return null;
            }

            @Override
            public void disengage() {

            }

            @Override
            public void engage() {

            }

            @Override
            public boolean isEngaged() {
                return false;
            }

            @Override
            public byte read8() {
                return 0;
            }

            @Override
            public byte read8(int ireg) {
                return 0;
            }

            @Override
            public byte[] read(int creg) {
                return new byte[0];
            }

            @Override
            public byte[] read(int ireg, int creg) {
                return new byte[0];
            }

            @Override
            public TimestampedData readTimeStamped(int creg) {
                return null;
            }

            @Override
            public TimestampedData readTimeStamped(int ireg, int creg) {
                return null;
            }

            @Override
            public void write8(int bVal) {

            }

            @Override
            public void write8(int ireg, int bVal) {

            }

            @Override
            public void write(byte[] data) {

            }

            @Override
            public void write(int ireg, byte[] data) {

            }

            @Override
            public void write8(int bVal, I2cWaitControl waitControl) {

            }

            @Override
            public void write8(int ireg, int bVal, I2cWaitControl waitControl) {

            }

            @Override
            public void write(byte[] data, I2cWaitControl waitControl) {

            }

            @Override
            public void write(int ireg, byte[] data, I2cWaitControl waitControl) {

            }

            @Override
            public void waitForWriteCompletions(I2cWaitControl waitControl) {

            }

            @Override
            public void enableWriteCoalescing(boolean enable) {

            }

            @Override
            public boolean isWriteCoalescingEnabled() {
                return false;
            }

            @Override
            public boolean isArmed() {
                return false;
            }

            @Override
            public void setI2cAddr(I2cAddr i2cAddr) {

            }

            @Override
            public I2cAddr getI2cAddr() {
                return null;
            }

            @Override
            public void setLogging(boolean enabled) {

            }

            @Override
            public boolean getLogging() {
                return false;
            }

            @Override
            public void setLoggingTag(String loggingTag) {

            }

            @Override
            public String getLoggingTag() {
                return "";
            }

            @Override
            public void setUserConfiguredName(@Nullable String name) {

            }

            @Override
            public String getUserConfiguredName() {
                return "";
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return "";
            }

            @Override
            public String getConnectionInfo() {
                return "";
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {

            }

            @Override
            public void close() {

            }

            @Override
            public void setHealthStatus(HealthStatus status) {

            }

            @Override
            public HealthStatus getHealthStatus() {
                return null;
            }

            @Override
            public void setI2cAddress(I2cAddr newAddress) {

            }

            @Override
            public I2cAddr getI2cAddress() {
                return null;
            }
        };
    }


    static class EmptySparkFunOtos extends SparkFunOTOS{
        public EmptySparkFunOtos() {
            super(getEmptyI2cDeviceSynch());
        }

        @Override
        public Pose2D getPosition() {
            return new Pose2D(72, 72, Math.toRadians(90));
        }
    }
}
