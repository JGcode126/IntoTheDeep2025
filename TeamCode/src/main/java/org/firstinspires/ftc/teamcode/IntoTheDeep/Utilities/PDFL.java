package org.firstinspires.ftc.teamcode.IntoTheDeep.Utilities;


import com.roboctopi.cuttlefish.utils.RingBuffer;

import java.util.Timer;

public class PDFL{
    private double kP, kD, kF, kL;
    private double deadzone;
    private double homedConstant;
    private boolean homed = false;

    private Timer timer = new Timer();
    private RingBuffer<Double> timeBuffer = new RingBuffer<Double>(3, 0.0);
    private RingBuffer<Double> errorBuffer = new RingBuffer<Double>(3, 0.0);


    public PDFL(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void updateConstant(double kP, double kD, double kF, double kL){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }

    public void setDeadzone(double deadzone){
        this.deadzone = deadzone;
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }

    public void setHomedConstant(double constant){
        homedConstant = constant;
    }

    public void reset(){
        timeBuffer.fill(0.0);
        errorBuffer.fill(0.0);
        timer.reset();
    }

    //function takes in error and gives back response
    public double run(double error){
        if(homed){
            return homedConstant;//id friving motor incorrectly can cause to motors burning out or robot restarting
        }

        double time = timer.getTime();
        double previousTime = timeBuffer.getValue(time);
        double previousError = errorBuffer.getValue(error);

        //if PDFL hasn't been updated, reset
        if(deltaTime > 200){//if delta time greater than 200 milliseconds
            reset();
            return run(error);
        }

        double deltaTime = time - previousTime;
        double deltaError = error - previousError;

        double p = pComponent(error);
        double d = dComponent(deltaError, deltaTime);
        double f = fComponent();
        double l = lComponent(error);

        double response = p + d + f + l;

        if(Math.abs(error) < deadzone){
            //same response but without lower limit
            response = p + d + f;
        }

        return response;
    }

    private double pComponent(double error){
        double response = kP * error;

        return response;
    }

    private double dComponent(double delta_error, double delta_time){
        double derivative = delta_error / delta_time;
        double response = derivative * kD;

        return response;
    }

    private double fComponent(){
        double response = kF;

        return response;
    }

    private double lComponent(double error){
        double direction = Math.signum(error);//gets direction of the error
        double response = direction * kL;

        return response;
    }
}
