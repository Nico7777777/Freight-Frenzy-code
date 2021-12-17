package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Iedi on 20.01.2017.
 */

public class GYRO extends Thread {
    private final Double reset = 0d;
    private final Double stillSpeed = 0d;
    public Double correctionSpeed = 0d;

    private boolean direction = true;
    private boolean isMoving = false;
    public boolean targetReached = false;

    public Double motorLeftPower = 0d;
    public Double motorRightPower = 0d;

    HardwareMap hardwareMap = null;
    GyroSensor gyroSensor = null;


    int heading = 0;
    int target  = 0;
    int error = 0;

    Double motorPower = 0d;
    GYRO(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        gyroSensor = this.hardwareMap.gyroSensor.get("gyro");
        this.start();
    }



    public void run(){
        heading = getHeading();
        error = getError();
        isMoving = getIsMoving();
        targetReached = isTargetReached();
        direction = getDirection();
        motorPower = getMotorPower();

    }

    private Double getMotorPower(){
        Double power;
        Double motorPower;
        power = Utils.range((double)error, 0d, 90d, 0d ,1d);
        if(direction == true){
            motorPower = power;
        }
        else{
            motorPower = -power;
        }
        return  motorPower;
    }

    public boolean isTargetReached(){
        if(Math.abs(error) <= 3 && isMoving == false){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean getIsMoving(){
        boolean isMoving = false;
        if(gyroSensor.getRotationFraction() < stillSpeed)
            isMoving = false;
        if(gyroSensor.getRotationFraction() > stillSpeed)
            isMoving = true;
        return isMoving;
    }

    public int getHeading() {
        int heading = gyroSensor.getHeading();
        if (heading > 180)
            heading = heading - 360;
        return heading;
    }

    public int getError(){
        int error;
        error = heading - target;
        error = Math.abs(error);
        return error;
    }

    public boolean getDirection(){
        //true -- clockwise
        //false -- counterclockwise
        if(error < 0)
            return true;
        else
            return false;
    }


    public void setTarget(int target){
        this.target = target;
    }

}
