package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

public class HardwareSky

{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    //public DcMotor  armDrive    = null;
    public Servo    left_hand ; // = null;
    public Servo    right_hand;//  = null;
    //public DcMotor  leftIntake  = null;
    public DcMotor  leftElv     = null;
    public DcMotor  rightElv    = null;
    //public Servo    dropper;
    public DcMotor  horiElv     = null;
    public DcMotor  leftIntake  = null;
    public DcMotor  rightIntake = null;
    public DcMotor  capstone    = null;
    public Servo    pickup;
   // public DistanceSensor sensorRange;
   // public ColorSensor sensorColor;
   // public TouchSensor sensorTouch;




    //DigitalChannel digitalTouch;

    /* public DcMotor  leftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public HardwareSky(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        //armDrive = hwMap.get(DcMotor.class, "arm_drive" );
        //leftIntake = hwMap.get(DcMotor.class, "left_intake");
        leftElv = hwMap.get(DcMotor.class, "left_elevator");
        rightElv = hwMap.get(DcMotor.class, "right_elevator");
        //gripDrive =hwMap.get(DcMotor.class,"gripper");
        //digitalTouch = hwMap.get(DigitalChannel.class, "sensor_digital");
        horiElv = hwMap.get(DcMotor.class, "horizontal_elevator");
        leftIntake = hwMap.get(DcMotor.class, "left_intake");
        rightIntake = hwMap.get(DcMotor.class, "right_intake");
        capstone = hwMap.get(DcMotor.class, "cap_stone");



        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //armDrive.setDirection(DcMotor.Direction.FORWARD);
        //leftIntake.setDirection(DcMotor.Direction.FORWARD);
        leftElv.setDirection(DcMotor.Direction.FORWARD);
        rightElv.setDirection(DcMotor.Direction.FORWARD);
        horiElv.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        capstone.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //armDrive.setPower(0);
        //leftIntake.setPower(0);
        leftElv.setPower(0);
        rightElv.setPower(0);
        //gripDrive.setpower(0);
        horiElv.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        capstone.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiElv.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capstone.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        left_hand  = hwMap.get(Servo.class, "left_hand");
        right_hand = hwMap.get(Servo.class, "right_hand");
        pickup = hwMap.get(Servo.class, "pick_up");
        left_hand.setPosition(0.8);
        right_hand.setPosition(0.2);
        pickup.setPosition(0.8);
    }
}

