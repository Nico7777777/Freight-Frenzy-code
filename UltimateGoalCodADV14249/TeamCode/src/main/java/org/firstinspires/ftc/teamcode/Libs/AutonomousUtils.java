package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by User on 28/01/2017.
 */

public class AutonomousUtils {
    Telemetry telemetry = null;
    HardwareMap hardwareMap = null;

    ElapsedTime runtime = new ElapsedTime();

    DcMotor motorStangaSpate = null;
    DcMotor motorStangaFata = null;

    DcMotor motorDreaptaSpate = null;
    DcMotor motorDreaptaFata = null;

    DcMotor motorMaturica = null;
    DcMotor motorLansatorStanga = null;
    DcMotor motorLansatorDreapta = null;

    Servo servoApasatorStanga = null;
    Servo servoApasatorDreapta = null;
    ColorSensor colorSensor = null;
    ColorSensor colorBurta = null;
    GyroSensor gyroSensor = null;
    VoltageSensor voltangeSensor = null;
    ModernRoboticsI2cRangeSensor rangeDreapta;
    ModernRoboticsI2cRangeSensor rangeStanga;
    public double voltagePower = 0.2;

    AutonomousUtils(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        gyroSensor = this.hardwareMap.gyroSensor.get("gyro");

        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");

        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");

        motorMaturica = hardwareMap.dcMotor.get("motorMaturica");
        motorLansatorDreapta = hardwareMap.dcMotor.get("motorLansatorDreapta");
        motorLansatorStanga = hardwareMap.dcMotor.get("motorLansatorStanga");

        colorSensor = this.hardwareMap.colorSensor.get("culoare");
        colorBurta = this.hardwareMap.colorSensor.get("culoareBurta");
        colorBurta.setI2cAddress(I2cAddr.create8bit(0x3a));
        voltangeSensor = this.hardwareMap.voltageSensor.get("lansator");

        servoApasatorStanga = this.hardwareMap.servo.get("servoApasatorStanga");
        servoApasatorDreapta = this.hardwareMap.servo.get("servoApasatorDreapta");

        rangeDreapta = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaDreapta");
        rangeStanga = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaStanga");
        rangeStanga.setI2cAddress(I2cAddr.create8bit(0x3e));

        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);

        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);

        /*resetEncoders();
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating());
        gyroSensor.resetZAxisIntegrator();
        telemetry.addData("gyro calibrat","");
        telemetry.update();
        waitForStart();
        gyroSensor.resetZAxisIntegrator();
        //launchBall();
        go(700,"f",0.3);
        goToDegr(-45);

        go(2500,"nobrake",0.8);
        goToWhiteLine();

        goToDegr(-90);
        goToWall(10);

        motorPower(0.2,0.2);
        waitSec(0.15);
        motorPower(0,0);
        waitSec(0.1);
        apasa("red");

        goBack(500,"break",0.2);
        goToDegr(0);
        go(1600,"nobrake",0.7);
        goToWhiteLine();

        goToDegr(-90);
        goToWall(10);
        motorPower(0.2,0.2);
        waitSec(0.15);
        motorPower(0,0);
        waitSec(0.1);
        apasa("red");

        goBack(400,"brake",0.3);
        goToDegr(-45);
        goBack(4000,"brake",1);*/
    }


    public void goToWall(int cm) {
        motorPower(0.3, 0.3);
        while(rangeDreapta.cmUltrasonic() > cm);
        motorPower(0,0);
    }

    private void goToWhiteLine() {
        motorPower(0.2,0.2);
        while(colorBurta.alpha() < 5) {

        }
        motorPower(0,0);
        waitSec(0.1);
    }

    public void launchBall(){
        motorLansare(0.3);
        waitSec(3);
        motorMaturica.setPower(1);
        waitSec(5);
        motorLansare(0);
        motorMaturica.setPower(0);
    }

    private void resetEncoders() {
        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void go (int ticks, String brake, double speed) {
        int pos = motorDreaptaFata.getCurrentPosition();
        ticks = ticks + pos;
        motorPower(speed, speed);
        while(pos < ticks) {
            pos = motorDreaptaFata.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        if(brake != "nobrake") {
            motorPower(0,0);
            waitSec(0.1);
        }
    }

    public void goBack (int ticks, String brake, double speed) {
        int pos = motorDreaptaFata.getCurrentPosition();
        ticks =pos - ticks;
        motorPower(-speed, -speed);
        while(pos > ticks) {
            pos = motorDreaptaFata.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        if(brake != "nobrake") {
            motorPower(0,0);
            waitSec(0.1);
        }
    }

    public void motorPower(double stg, double drt) {
        motorStangaSpate.setPower(stg);
        motorStangaFata.setPower(stg);
        motorDreaptaSpate.setPower(drt);
        motorDreaptaFata.setPower(drt);
    }

    public void motorLansare(double speed) {
        motorLansatorDreapta.setPower(speed);
        motorLansatorStanga.setPower(speed);
    }



    public void apasa(String color){
        /*
        if(getColor().compareToIgnoreCase("blue") == 0){
            servoApasatorDreapta.setPosition(0);
            servoApasatorStanga.setPosition(1);
        }
        else{
            servoApasatorDreapta.setPosition(1);
            servoApasatorStanga.setPosition(0);
        }
        waitSec(1);
        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);*/


        if(getColor().compareToIgnoreCase("blue") == 0){
            if(color.compareToIgnoreCase("blue") == 0) {
                servoApasatorDreapta.setPosition(1);
                servoApasatorStanga.setPosition(0);
            }
            else{
                servoApasatorDreapta.setPosition(0);
                servoApasatorStanga.setPosition(1);
            }
        }
        else{
            if(color.compareToIgnoreCase("blue") == 0) {
                servoApasatorDreapta.setPosition(0);
                servoApasatorStanga.setPosition(1);
            }
            else{
                servoApasatorDreapta.setPosition(1);
                servoApasatorStanga.setPosition(0);
            }
        }
        waitSec(1);
        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);

    }

    public String getColor(){
        if(colorSensor.blue() > colorSensor.red())
            return "blue";
        else
            return "red";
    }

    public void goToDegr(int target) {
        double targetCorrection;
        double correctionPower;
        if(voltangeSensor.getVoltage() > 12) {
            voltagePower = 0.15;
            targetCorrection = 10;
        }
        else {
            voltagePower = 0.3;
            targetCorrection = 0;
        }

        if(voltangeSensor.getVoltage() > 13.5) {
            correctionPower = 0.03;
        } else {
            correctionPower = 0.05;
        }

        showGyroDebug();
        if(target - getHeading() >= 0){
            motorPower(voltagePower, -voltagePower);
            while(getHeading() < (target - 20 - targetCorrection)) {
                showGyroDebug();
            }
            motorPower(0.05, -0.05);
            while(getHeading() < (target - targetCorrection)) {
                showGyroDebug();
            }
            motorPower(0, 0);
        }else{
            motorPower(-voltagePower, voltagePower);
            while(getHeading() > (target + 20 + targetCorrection)) {
                showGyroDebug();
            }
            motorPower(-0.05, 0.05);
            while(getHeading() > (target + targetCorrection)) {
                showGyroDebug();
            }
            motorPower(0, 0);
        }
        waitSec(0.3);
        while(Math.abs(getHeading() - target) > 3){
            if(target - getHeading() >= 0){
                if(getHeading() < (target + 3)){
                    motorPower(correctionPower,-correctionPower);
                    while(getHeading() < (target - 3)) {
                        showGyroDebug();
                    }
                    motorPower(0,0);
                }
                else if(getHeading() > (target + 3)) {
                    motorPower(-correctionPower,correctionPower);
                    while(getHeading() > (target + 3)) {
                        showGyroDebug();
                    }
                    motorPower(0,0);
                }
            } else {
                if(getHeading() < (target + 3)){
                    motorPower(correctionPower,-correctionPower);
                    while(getHeading() < (target - 3)) {
                        showGyroDebug();
                    }
                    motorPower(0,0);
                }
                else if(getHeading() > (target + 3)) {
                    motorPower(-correctionPower,correctionPower);
                    while(getHeading() > (target + 3)) {
                        showGyroDebug();
                    }
                    motorPower(0,0);
                }
            }
            waitSec(0.3);
        }
    }

    public int getHeading() {
        int heading = gyroSensor.getHeading();
        if (heading > 180)
            heading = heading - 360;
        return heading;
    }

    public void showGyroDebug() {
        telemetry.addData("gyro", getHeading());
        telemetry.update();
    }

    public void  waitSec(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) showGyroDebug();
    }

    public void runToPosition(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() + ticks;
        setTargetPosition(ticks);
        motorPower(0.7, 0.7);
        while(isBusy());
        motorPower(0, 0);
        motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPositionCoast(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() + ticks;
        setTargetPosition(ticks);
        motorPower(0.7, 0.7);
        while(isBusy());
        motorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motorPower(0, 0);

    }

    public void runToPositionBack(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() - ticks;
        setTargetPosition(ticks);
        motorPower(-0.7, -0.7);
        while(isBusy());
        //motorPower(0, 0);
        motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusy(){
        return motorStangaFata.isBusy() || motorStangaSpate.isBusy() || motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy();
    }

    public void motorMode(DcMotor.RunMode runmode){
        motorStangaFata.setMode(runmode);
        motorStangaSpate.setMode(runmode);
        motorDreaptaFata.setMode(runmode);
        motorDreaptaSpate.setMode(runmode);
    }

    public void setTargetPosition(int ticks){
        motorStangaFata.setTargetPosition(ticks);
        motorStangaSpate.setTargetPosition(ticks);

        motorDreaptaFata.setTargetPosition(ticks);
        motorDreaptaSpate.setTargetPosition(ticks);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        motorStangaFata.setZeroPowerBehavior(zeroPowerBehavior);
        motorStangaSpate.setZeroPowerBehavior(zeroPowerBehavior);

        motorDreaptaFata.setZeroPowerBehavior(zeroPowerBehavior);
        motorDreaptaSpate.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
