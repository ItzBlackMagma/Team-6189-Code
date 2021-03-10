package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    public DcMotor spin,load,angle;
    public Servo push;

    public final double FIRE_POS = 1, RELOAD_POS = .8, ANGLE_CPR = 14, MAX_ANGLE = 45, DegPerPulse = 360 / ANGLE_CPR, PulsesPerDeg = ANGLE_CPR / 360;
    private double power = 0;
    private double lastSpinPos = 0;
    private double spinSpeed = 0;

    public void init(HardwareMap hm){
        load = hm.dcMotor.get("loader");
        spin = hm.dcMotor.get("right spin");
        angle = hm.dcMotor.get("angle");
        push = hm.get(Servo.class, "push");

        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        push.setDirection(Servo.Direction.FORWARD);

        spin.setDirection(DcMotorSimple.Direction.REVERSE);
        load.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reload();
    }

    public void fire(){push.setPosition(FIRE_POS);}

    public void reload(){push.setPosition(RELOAD_POS);}

    public void spinPower(double power){spin.setPower(power);}

    public void loadPower(double power){load.setPower(power);}

    public void updateSpinSpeed(long time){
        double theta = ((spin.getCurrentPosition() - lastSpinPos) / 28) * 360;
        lastSpinPos = spin.getCurrentPosition();
        spinSpeed = (2 * theta) / (time * 1000); // 2 is the radius of the flywheel and is measured in inches
    }

    public void rotateToPos(double pos, int inv){
        // angle.setTargetPosition((int)pos);
        angle.setPower(power);

//        if(angle.getCurrentPosition() < pos){
//            power *= inv;
//        } else if(angle.getCurrentPosition() > pos){
//            power *= -inv;
//        } else {
//            power = 0;
//        }
//        angle.setPower(power);
    }

    public void rotateToAngle(double angle, int inv){rotateToPos(angle * PulsesPerDeg, inv);}

    public double getAngle(){return  angle.getCurrentPosition() * DegPerPulse;}

    public void setAnglePower(double power){this.power = power;}

    public double getSpinSpeed() {
        return spinSpeed;
    }

    public void setSpinSpeed(double spinSpeed, double power) {
        power *= (spinSpeed - this.spinSpeed) > 0 ? 1 : -1;
        spin.setPower(power);
    }
}
