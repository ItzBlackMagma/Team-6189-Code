package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {
    DcMotor lifter;
    Servo gripper;
    // CRServo extender;

    final double COUNTS_PER_INCH = 5;  // needs to be replaced with the actual number

    public void init(HardwareMap hm){
        lifter = hm.dcMotor.get("wobble");
        gripper = hm.servo.get("lock");
        // extender = hm.get(CRServo.class, "linear");

        lifter.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gripper.setDirection(Servo.Direction.FORWARD);
        // extender.setDirection(DcMotorSimple.Direction.FORWARD);
        resetEncoders();
        noEncoders();
    }

    public void lift(double power){
        lifter.setPower(power);
    }

    public void raiseToPos(double pos, double power){
        lifter.setPower(power);
        lifter.setTargetPosition((int) pos); // (pos / COUNTS_PER_INCH));
        toPosition();
    }

    // public void extend(double power){ extender.setPower(power); }

    public void grip(double pos){
        gripper.setPosition(pos);
    }

    public void toPosition(){
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoders(){
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void noEncoders(){
        lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void useEncoders(){
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
