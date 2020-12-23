package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

    Telemetry telemetry;
    //public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Crossbow module
    public DcMotor sideways, vertical, drawback, leftSpin, rightSpin;
    public DcMotor motor1,motor2,motor3,motor4; //starts with left front and moves clockwise
    //public DcMotor verticalLeft, verticalRight, horizontal; //Odometry motors

    public double robotX, robotY, robotAngle;

    public boolean atAngle = false;
    public boolean atPoint = false;

    public Servo lock;

    //Chassis module
    public WebcamName webcamName;

    public BNO055IMU imu;

    // We use millimeters for accuracy
    public final double COUNTS_PER_INCH = Locations.mmPerInch * 5; // calculate this with a real number
    public final double COUNTS_PER_REV = COUNTS_PER_INCH;
    public double launchHeight = Locations.mmPerInch * 18;

    public RobotHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void move(double x, double y, double r, double power){

        double power1 = y + x + r;
        double power2 = y - x - r;
        double power3 = y + x - r;
        double power4 = y - x + r;

        motor1.setPower(power1 * power);
        motor2.setPower(power2 * power);
        motor3.setPower(power3 * power);
        motor4.setPower(power4 * power);

        telemetry.addData("/> IMU", getRotation("Z"));

    }

    public void stop(){
        move(0,0,0,0);
    }

    public void moveToPoint(double x, double y, double r, double power, double error){ // takes in inches
        double deltaX = x - robotX;
        double deltaY = y - robotY;

        boolean atX = false;
        boolean atY = false;

        if (deltaX > error) {
            move(1, 0, r, power);
        } if (deltaX < -error) {
            move(-1, 0, r, power);
        } else {
            atX = true;
        }

        if (deltaY > error) {
            move(0, 1, r, power);
        } if (deltaY < -error) {
            move(0, -1, r, power);
        } else {
            atY = true;
        }


        if (atX && atY) {
            atPoint = true;
            stop();
        }

        telemetry.addData("/> AtPoint", atPoint);
    }

    public void moveToPoint(double x, double y, double power, double error) { // takes in inches
        moveToPoint(x, y, power, error);
    }

    public void autoToPoint(double x, double y, double power, double error, boolean opModeIsActive){ // specifically for autonomous period
        atPoint = false;
        while (!atPoint && opModeIsActive){
            moveToPoint(x, y, power, error);
        }
    }


    public double getPowerToAngle(double angle){
        double currentAngle = getRotation("Z");
        double deltaAngle = angle - currentAngle;

        if (deltaAngle > 0) {
            atAngle = false;
            return 1.0;
        } else if (deltaAngle < 0) {
            atAngle = false;
            return -1.0;
        } else {
            atAngle = true;
            return 0;
        }

    }

    public double getAngleToPoint(double pointX, double pointY){ // takes in inches
        double desiredAngle = Math.atan2(pointX - robotX, pointY - robotY);
        return desiredAngle;
    }

    public double getRotation(String axis){
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        switch(axis) {
            case "Z":
                return angles.thirdAngle;
            case "Y":
                return angles.secondAngle;
            case "X":
                return angles.firstAngle;
            default:
                return 0;
        }
    }

    public Team getTeam(Camera camera){
        if (robotY > 0){
            return Team.BLUE;
        } else {
            return Team.RED;
        }
    }

    public void setLaunchPower(double power){
        rightSpin.setPower(power);
        leftSpin.setPower(power);
    }

    public double calculateUp(double[] pos, int inv){
        double x = pos[0];
        double y = pos[1] * inv;
        double z = pos[2];

        double distance = Math.hypot(x - robotX, y - robotY);
        double upAngle = Math.atan2(z, distance);

        return upAngle;
    }

    public void aim(double[] pos, int inv, boolean rotate){
        double upAngle = calculateUp(pos, inv);
        double sideAngle = getAngleToPoint(pos[0], pos[1] * inv);

        if(rotate)
            move(0, 0, getPowerToAngle(sideAngle), 0.75);

        // vertical.setPower();

        telemetry.addData("/> SYSTEM", "AIMBOT IS RUNNING");

    }

    public int getInvFromTeam(Team team){
        switch (team){
            case RED:
                return -1;
            case BLUE:
                return 1;
            default:
                return 0;
        }
    }

    //Set everything up
    public void init(HardwareMap hardwareMap){
        telemetry.addData("/> SYSTEM", "Beginning Initialization Process...");

        initIMU(hardwareMap);

        initChassis(hardwareMap);
        initLauncher(hardwareMap);

        //initBow(hardwareMap);
        //initServos(hardwareMap);

        //webcamName = hardwareMap.get(WebcamName.class, "robo eye");

        telemetry.addData("/> INIT", "Webcam Initialized...");
        telemetry.addData("/> SYSTEM", "Initialization Complete");
        telemetry.update();
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("/> INIT", "Imu Initialized");

    }

    private void initLauncher(HardwareMap hardwareMap){
        leftSpin = hardwareMap.dcMotor.get("left spin");
        rightSpin = hardwareMap.dcMotor.get("right spin");

        rightSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSpin.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initChassis(HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("/> INIT", "Chassis Motors Initialized");
    }

    private void initBow(HardwareMap hardwareMap){
        sideways = hardwareMap.dcMotor.get("vertical");
        vertical = hardwareMap.dcMotor.get("horizontal");
        drawback = hardwareMap.dcMotor.get("drawback");

        sideways.setDirection(DcMotor.Direction.FORWARD);
        vertical.setDirection(DcMotor.Direction.FORWARD);
        drawback.setDirection(DcMotor.Direction.FORWARD);

        sideways.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drawback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders
        sideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drawback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("/> INIT", "Motors Initialized...");
        telemetry.update();
    }

    private void initServos(HardwareMap hardwareMap){
        lock = hardwareMap.servo.get("lock");
        telemetry.addData("/> INIT", "Servos Initialized...");
        telemetry.update();
    }

    //public void stopGPS(){ globalPositionUpdate.stop(); }
    
}
