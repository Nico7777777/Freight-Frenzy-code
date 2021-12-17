package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by edidi on 21.10.2016.
 */
public class GAMEPAD{
    Telemetry telemetry = null;
    public Double left_stick_x = 0.0;
    public Double left_stick_y = 0.0;

    public Double right_stick_x = 0.0;
    public Double right_stick_y = 0.0;

    public Button dpad_up = new Button();
    public Button dpad_down = new Button();
    public Button dpad_left = new Button();
    public Button dpad_right = new Button();

    public Double dpad_power = 0.0;

    public Button a = new Button();
    public Button b = new Button();
    public Button x = new Button();
    public Button y = new Button();

    public int maturicaDirection = 0;

    public boolean guide = false;
    public boolean start = false;
    public boolean back = false;

    public Button left_bumper = new Button();
    public Button right_bumper = new Button();

    public Button left_stick_button = new Button();
    public Button right_stick_button = new Button();

    public Double left_trigger = 0.0;
    public Double right_trigger = 0.0;

    public Double left_stick_angle = 0.0;
    public Double right_stick_angle = 0.0;

    public Double left_stick_degrees = 0.0;
    public Double right_stick_degrees = 0.0;

    public int left_stick_quadrant = 1;
    public int right_stick_quadrant = 1;

    public Double left_stick_powerX = 0.0;
    public Double left_stick_powerY = 0.0;

    public Double right_stick_powerX = 0.0;
    public Double right_stick_powerY = 0.0;

    boolean gamepad_type = false;
    private Gamepad gamepad = null;

    private RealAxes leftAxes = new RealAxes();
    private RealAxes rightAxes = new RealAxes();

    public GAMEPAD(Gamepad gamepad, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.gamepad_type = false;
        this.run();
    }

    /*public GAMEPAD(Gamepad gamepad, Telemetry telemetry, String gamepad_type) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        if (gamepad_type.compareToIgnoreCase("logitech") == 0)
            this.gamepad_type = false;
        if (gamepad_type.compareToIgnoreCase("xbox") == 0)
            this.gamepad_type = true;
        this.start();
    }*/

    public void run() {
        update();
    }

    private void update(){
        updateMaturicaToggle();
        updateDpadPower();

        this.guide = gamepad.guide;
        this.start = gamepad.start;
        this.back = gamepad.back;

        this.left_stick_x = (double)gamepad.left_stick_x;
        this.left_stick_y = (double)-gamepad.left_stick_y;

        this.right_stick_x = (double)gamepad.right_stick_x;
        this.right_stick_y = (double)-gamepad.right_stick_y;

        this.left_trigger =(double)gamepad.left_trigger;
        this.right_trigger =(double)gamepad.right_trigger;

        this.a.updateValue(gamepad.a);
        this.b.updateValue(gamepad.b);
        this.x.updateValue(gamepad.x);
        this.y.updateValue(gamepad.y);

        this.dpad_up.updateValue(gamepad.dpad_up);
        this.dpad_down.updateValue(gamepad.dpad_down);
        this.dpad_left.updateValue(gamepad.dpad_left);
        this.dpad_right.updateValue(gamepad.dpad_right);

        this.left_bumper.updateValue(gamepad.left_bumper);
        this.right_bumper.updateValue(gamepad.right_bumper);

        this.left_stick_button.updateValue(gamepad.left_stick_button);
        this.right_stick_button.updateValue(gamepad.right_stick_button);

        leftAxes.calculate(left_stick_x, left_stick_y);
        rightAxes.calculate(right_stick_x, right_stick_y);

        left_stick_angle = leftAxes.angle;
        right_stick_angle = rightAxes.angle;

        left_stick_quadrant = leftAxes.quadrant;
        right_stick_quadrant = rightAxes.quadrant;

        left_stick_degrees = leftAxes.degrees;
        right_stick_degrees = rightAxes.degrees;

        left_stick_powerX = leftAxes.powerX;
        left_stick_powerY = leftAxes.powerY;

        right_stick_powerX = rightAxes.powerX;
        right_stick_powerY = rightAxes.powerY;

        this.left_stick_angle = angle(this.left_stick_x, this.left_stick_y);
        this.right_stick_angle = angle(this.right_stick_x, this.right_stick_y);

        this.left_stick_quadrant = quadrant(this.left_stick_angle);
        this.right_stick_quadrant = quadrant(this.right_stick_angle);

        /*telemetry.addData("left angle: ", left_stick_angle);
        telemetry.addData("right angle: ", right_stick_angle);

        telemetry.addData("left quadrant: ", left_stick_quadrant);
        telemetry.addData("right quadrant: ", right_stick_quadrant);

        telemetry.addData("left degree: ", left_stick_degrees);
        telemetry.addData("right degree: ", right_stick_degrees);

        telemetry.addData("left powerX: ", left_stick_powerX);
        telemetry.addData("left powerY: ", left_stick_powerY);

        telemetry.addData("right powerX: ", right_stick_powerX);
        telemetry.addData("right powerY: ", right_stick_powerY);*/

        /*telemetry.addData("y.toggle: ", y.toggle);
        telemetry.addData("y: ", gamepad.y);
        telemetry.addData("y value", y.value);
        telemetry.addData("y last value: ", y.lastValue);
        telemetry.addData("y intrat: ", y.intrat);*/
    }

    private void updateDpadPower(){
        if(gamepad.dpad_up == true){
            dpad_power = 0.2d;
        }
        if(gamepad.dpad_down == true){
            dpad_power = -1d;
        }
        if(gamepad.dpad_up == false && gamepad.dpad_down == false){
            dpad_power = 0d;
        }
    }


    private void updateMaturicaToggle(){
        if(x.toggle == true && (x.lastToggleTime > b.lastToggleTime)){
            b.toggle = false;
            maturicaDirection = 1;
        }
        if(b.toggle == true && (b.lastToggleTime > x.lastToggleTime)){
            x.toggle = false;
            maturicaDirection = -1;
        }
        if(x.toggle == false && b.toggle == false){
            maturicaDirection = 0;
        }

    }

    class RealAxes{
        Double realX;
        Double realY;
        Double angle;
        Double degrees;
        Double powerX;
        Double powerY;
        Double radius;
        int quadrant;

        private void calculate(Double stick_x, Double stick_y){
            angle(stick_x, stick_y);
            realX();
            realY();
            power();
        }

        private void power(){
            /*telemetry.addData("realX: ", realX);
            telemetry.addData("realY: ", realY);
            telemetry.addData("radius: ", radius);*/
            this.powerX = realX * radius;
            this.powerY = realY * radius;
        }



        private void realX(){
            Double realx = 0.0;
            if(quadrant == 1 || quadrant == 2)
                realx = Utils.scale(angle, 0.000, 180.0,  1.0, -1.0);
            if(quadrant == 3 || quadrant == 4)
                realx = Utils.scale(angle, 180.0, 360.0, -1.0,  1.0);

            if(realx.isNaN() == true)
                realX = 0.0;
            else
                realX = realx;
        }

        private void realY(){
            Double realy = 0.0;
            if(quadrant == 1)
                realy = Utils.scale(angle, 0.000, 90.00,  0.0,  1.0);
            if(quadrant == 2 || quadrant == 3)
                realy = Utils.scale(angle, 90.00, 270.0,  1.0, -1.0);
            if(quadrant == 4)
                realy = Utils.scale(angle, 270.0, 360.0, -1.0,  0.0);

            if(realy.isNaN() == true)
                realY = 0.0;
            else
                realY = realy;
        }

        private void angle(Double stick_x, Double stick_y) {
            if (gamepad_type == false)
                angle_logitech(stick_x, stick_y);
            if (gamepad_type == true)
                angle_xbox(stick_x, stick_y);
        }


        private void angle_logitech(Double stick_x, Double stick_y) {
            Double x_axis = stick_x;
            Double y_axis = stick_y;

            Double hypotenuse = 0.0;
            Double cosine = 0.0;
            Double acos = 0.0;

            int logitechQuadrant = 1;
            Double logitechDegrees = 0.0;
            Double logitechAngle = 0.0;

            hypotenuse = Math.abs(x_axis) * Math.abs(x_axis);
            hypotenuse += Math.abs(y_axis) * Math.abs(y_axis);
            hypotenuse = Math.sqrt(hypotenuse);

            cosine = Math.abs(x_axis) / hypotenuse;
            acos = Math.acos(cosine);
            logitechDegrees = Math.toDegrees(acos);

            if (y_axis >= 0) {
                if (x_axis >= 0)
                    logitechQuadrant = 1;
                else
                    logitechQuadrant = 2;
            } else {
                if (x_axis <= 0)
                    logitechQuadrant = 3;
                else
                    logitechQuadrant = 4;
            }

            if (logitechQuadrant == 1)
                logitechAngle = 0 + logitechDegrees;
            if (logitechQuadrant == 2)
                logitechAngle = 180 - logitechDegrees;
            if (logitechQuadrant == 3)
                logitechAngle = 180 + logitechDegrees;
            if (logitechQuadrant == 4)
                logitechAngle = 360 - logitechDegrees;

            if(Math.abs(x_axis) > Math.abs(y_axis))
                radius = x_axis;
            else
                radius = y_axis;
            radius = Math.abs(radius);

            quadrant = logitechQuadrant;
            degrees = logitechDegrees;
            angle = logitechAngle;


        }

        /*private void angle_xbox(Double stick_x, Double stick_y) {
            Double x_axis = stick_x;
            Double y_axis = stick_y;

            int xboxQuadrant = 1;
            Double xboxDegrees = 0.0;
            Double xboxAngle = 0.0;

            Double cosine = 0.0;
            Double acos = 0.0;
            Double hypotenuse = 0.0;

            hypotenuse = Math.abs(x_axis) * Math.abs(x_axis);
            hypotenuse += Math.abs(y_axis) * Math.abs(y_axis);
            hypotenuse = Math.sqrt(hypotenuse);

            cosine = Math.abs(x_axis);
            acos = Math.acos(cosine);
            xboxDegrees = Math.toDegrees(acos);

            if (y_axis >= 0) {
                if (x_axis >= 0)
                    xboxQuadrant = 1;
                else
                    xboxQuadrant = 2;
            } else {
                if (x_axis <= 0)
                    xboxQuadrant = 3;
                else
                    xboxQuadrant = 4;
            }

            if (xboxQuadrant == 1)
                xboxAngle = 0 + xboxDegrees;
            if (xboxQuadrant == 2)
                xboxAngle = 180 - xboxDegrees;
            if (xboxQuadrant == 3)
                xboxAngle = 180 + xboxDegrees;
            if (xboxQuadrant == 4)
                xboxAngle = 360 - xboxDegrees;


            radius = hypotenuse;
            radius = Math.abs(radius);
            quadrant = xboxQuadrant;
            degrees = xboxDegrees;
            angle = xboxAngle;
        }*/
    }

    public int quadrant(Double angle){
        int quadrant;
        quadrant = (int)(angle + 1);
        quadrant /= 90;
        quadrant = quadrant + 1;
        return quadrant;
    }

    public Double degree(int quadrant, Double angle){

        Double degree = (double)0;
        if(quadrant == 1)
            degree = angle;
        if(quadrant == 2)
            degree = 180 - angle;
        if(quadrant == 3)
            degree = angle - 180;
        if(quadrant == 4)
            degree = 360 - angle;

        return degree;
    }

    public Double angle(Double stick_x, Double stick_y) {
        Double angle = (double)0;
        if (gamepad_type == false)
            angle = angle_logitech(stick_x, stick_y);
        if (gamepad_type == true)
            angle = angle_xbox(stick_x, stick_y);
        return angle;
    }



    private Double angle_logitech(Double stick_x, Double stick_y) {
        Double x_axis = stick_x;
        Double y_axis = stick_y;

        Double hypotenuse = 0.0;
        int quadrant = 1;

        Double cosine = 0.0;
        Double acos = 0.0;

        Double degrees = 0.0;
        Double angle = 0.0;

        hypotenuse = Math.abs(x_axis) * Math.abs(x_axis);
        hypotenuse += Math.abs(y_axis) * Math.abs(y_axis);
        hypotenuse = Math.sqrt(hypotenuse);

        cosine = Math.abs(x_axis) / hypotenuse;
        acos = Math.acos(cosine);
        degrees = Math.toDegrees(acos);

        if (y_axis >= 0) {
            if (x_axis >= 0)
                quadrant = 1;
            else
                quadrant = 2;
        } else {
            if (x_axis <= 0)
                quadrant = 3;
            else
                quadrant = 4;
        }

        if (quadrant == 1)
            angle = 0 + degrees;
        if (quadrant == 2)
            angle = 180 - degrees;
        if (quadrant == 3)
            angle = 180 + degrees;
        if (quadrant == 4)
            angle = 360 - degrees;

        return angle;
    }

    private Double angle_xbox(Double stick_x, Double stick_y) {
        Double x_axis = stick_x;
        Double y_axis = stick_y;

        int quadrant = 1;

        Double cosine = 0.0;
        Double acos = 0.0;

        Double degrees = 0.0;
        Double angle = 0.0;

        cosine = Math.abs(x_axis);
        acos = Math.acos(cosine);
        degrees = Math.toDegrees(acos);

        if (y_axis >= 0) {
            if (x_axis >= 0)
                quadrant = 1;
            else
                quadrant = 2;
        } else {
            if (x_axis <= 0)
                quadrant = 3;
            else
                quadrant = 4;
        }

        if (quadrant == 1)
            angle = 0 + degrees;
        if (quadrant == 2)
            angle = 180 - degrees;
        if (quadrant == 3)
            angle = 180 + degrees;
        if (quadrant == 4)
            angle = 360 - degrees;

        return angle;
    }

}