package org.firstinspires.ftc.teamcode.SampleDetector;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public final class toPoseSample {

   public static class servoPoses
    {
        public double basePose, arm1Pose,arm2Pose,tipPose,swivelPose;
    }

    public static class minMax
    {
        double min, max;

    }
    public static minMax
            arm1Pose=new minMax(),
            arm2Pose=new minMax(),
            tipPose=new minMax(),
            swivelPose=new minMax(),
            basePose=new minMax();

   public static double arm1Horizontal = 0.23;

    public double clip(minMax limit, double check){

    return Math.max(limit.min, Math.min(limit.max, check));

    }
    public void limitCheck()
    {
        crtPose.tipPose=clip(tipPose,crtPose.tipPose);
        crtPose.arm1Pose=clip(arm1Pose,crtPose.arm1Pose);
        crtPose.arm2Pose=clip(arm2Pose,crtPose.arm2Pose);
        crtPose.swivelPose=clip(swivelPose,crtPose.swivelPose);
        crtPose.basePose=clip(basePose,crtPose.basePose);
    }
    public Servo baseServo,arm1Servo,arm2Servo,clawServo,tipServo,swivelServo;

    public static final double NumberOfSteps =40;
    private double n = NumberOfSteps;
    public static final long delayTime =40;
    private long delay= delayTime;
    public double clawPose =0.0;

    public static  double
     openClaw =0.4,
     closeClaw =0.1,

    deliverPoseRBase =0.57,
    deliverPoseRArm1 =0.38,
    deliverPoseRArm2 =0.62,
    deliverPoseRTip =0.1,
    deliverPoseRSwivel =0.5,

    deliverPoseBBase =0.67,
    deliverPoseBArm1 =0.61,
    deliverPoseBArm2 =0.88,
    deliverPoseBTip =0.1,
    deliverPoseBSwivel =0.5,

    homePoseBase =0.7,
    homePoseArm1 =0.95,

    homePoseArm2 =1,
    homePoseTip =0.1,
    homePoseSwivel =0.5,

    standPoseBase =0.8,
    standPoseArm1 =0.95,
    standPoseArm2 =1,
    standPoseTip =0.1,
    standPoseSwivel =0.5;

 public static servoPoses crtPose = new servoPoses();
 public servoPoses homePose = new servoPoses(),
         deliverPoseRed = new servoPoses(),
         deliverPoseBlue = new servoPoses(),
         deliveryIntermittent = new servoPoses(),
         pickPose = new servoPoses(),
         error = new servoPoses();

    private LinearOpMode op;

    public toPoseSample(LinearOpMode p_op) {
        op = p_op;
    }
    public void initialize()
    {
    // Connect to servo (Assume Robot Left Hand)
    // Change the text in quotes to match any servo name on your robot.
    baseServo = op.hardwareMap.get(Servo.class, "Base");
    arm1Servo = op.hardwareMap.get(Servo.class, "Arm1");
    arm2Servo = op.hardwareMap.get(Servo.class, "Arm2");
    clawServo = op.hardwareMap.get(Servo.class, "Claw");
    tipServo = op.hardwareMap.get(Servo.class, "ClawTip");
    swivelServo = op.hardwareMap.get(Servo.class, "ClawSpin");

        crtPose.arm1Pose = homePoseArm1;
        crtPose.arm2Pose = homePoseArm2;
        crtPose.basePose = homePoseBase;
        crtPose.swivelPose = homePoseSwivel;
        crtPose.tipPose = homePoseTip;
        clawPose = closeClaw;

        basePose.min = 0.15;
        basePose.max = 0.8;
        arm1Pose.min=0.01;
        arm1Pose.max=0.95;
        arm2Pose.min=0.01;
        arm2Pose.max=0.95;
        tipPose.min=0.03;
        tipPose.max=0.75;
        swivelPose.min=0.01;
        swivelPose.max=0.99;

     //clawServo.setPosition(closeClaw);
        home();
    }

    public void asignStaticData()
    {
        error.arm1Pose = 0;
        error.arm2Pose =0;
        error.basePose =0;
        error.tipPose =0;
        error.swivelPose=0;

        homePose.arm1Pose = homePoseArm1;
        homePose.arm2Pose = homePoseArm2;
        homePose.basePose = homePoseBase;
        homePose.swivelPose = homePoseSwivel;
        homePose.tipPose = homePoseTip;

        deliverPoseRed.arm1Pose = deliverPoseRArm1;
        deliverPoseRed.arm2Pose = deliverPoseRArm2;
        deliverPoseRed.basePose = deliverPoseRBase;
        deliverPoseRed.tipPose = deliverPoseRTip;
        deliverPoseRed.swivelPose = deliverPoseRSwivel;

        deliverPoseBlue.arm1Pose = deliverPoseBArm1;
        deliverPoseBlue.arm2Pose = deliverPoseBArm2;
        deliverPoseBlue.basePose = deliverPoseBBase;
        deliverPoseBlue.tipPose = deliverPoseBTip;
        deliverPoseBlue.swivelPose = deliverPoseBSwivel;

        deliveryIntermittent.arm1Pose = 0.55;
        deliveryIntermittent.arm2Pose = 0.6;
        deliveryIntermittent.basePose = .81;
        deliveryIntermittent.tipPose = 0.1;
        deliveryIntermittent.swivelPose = 0.5;

        pickPose.basePose = standPoseBase;
        pickPose.arm1Pose = standPoseArm1;
        pickPose.arm2Pose = standPoseArm2;
        pickPose.tipPose = standPoseTip;
        pickPose.swivelPose = standPoseSwivel;

    }
    public void deliverRed()
    {
        deliveryIntermittent.basePose=crtPose.basePose;
        toPose(deliveryIntermittent);
        toPose(deliverPoseRed);
        setOpenClaw();
    }
    public void deliverBlue()
    {
        deliveryIntermittent.basePose=crtPose.basePose;
        toPose(deliveryIntermittent);
        toPose(deliverPoseBlue);
        setOpenClaw();
    }

    public void runNoNo() {

        for (int l = 0; l < 2; l++) {
            swivelServo.setPosition(0.5 + 0.4);
            op.sleep(200);
            swivelServo.setPosition(0.5 - 0.4);
            op.sleep(200);
            swivelServo.setPosition(0.5);
        }

    }

    public Interpolate interpol = new Interpolate();

    public void findPose(double angle){
        pickPose.basePose = interpol.basePose();
        pickPose.arm1Pose = interpol.arm1Pose();
        pickPose.arm2Pose = interpol.arm2Pose();
        pickPose.tipPose = interpol.tipPose();

        int sign = 1;

        if(angle<0) { sign = -1;}
            pickPose.swivelPose = 0.5-interpol.swivelPose()/180 - sign * ((90 - Math.abs(angle)) / 180);

        if(pickPose.swivelPose<0.0)
        {
            pickPose.swivelPose=1.0+pickPose.swivelPose;
        }else if(pickPose.swivelPose>1.0){
            pickPose.swivelPose= pickPose.swivelPose -1;
        }

    }

    public boolean pickSample(double x, double y, double angle) {

        boolean noNo = interpol.findSample(x,y);
        pickPose.basePose = interpol.basePose();
        toPose(pickPose);

        if(!noNo){
            runNoNo();
            return false;
        }else {
            findPose(angle);
            toPose(pickPose);
            setCloseClaw();
        }

        return true;
    }

    public void home()
    {
        asignStaticData();
        toPose(homePose);
        setOpenClaw();
    }

    //function to find difference in position
    public void findError(servoPoses poses)
    {
        error.basePose = poses.basePose - crtPose.basePose;
        error.arm1Pose=poses.arm1Pose- crtPose.arm1Pose;
        error.arm2Pose=poses.arm2Pose- crtPose.arm2Pose;
        error.tipPose=poses.tipPose- crtPose.tipPose;
        error.swivelPose=poses.swivelPose- crtPose.swivelPose;
    }

    public void setCloseClaw()
    {

     if(clawPose==openClaw) {
      clawPose = closeClaw;
      clawServo.setPosition(closeClaw);
       }
    }

     public void setOpenClaw()
     {

      if(clawPose==closeClaw) {
       clawPose = openClaw;
       clawServo.setPosition(clawPose);

      }
     }

    // function asks robot to move to position
    public void toPose(servoPoses poses)
    {
        findError(poses);

        for (int i=0;i<n;i++)
        {
            crtPose.basePose=crtPose.basePose+(error.basePose/n);
            crtPose.arm1Pose=crtPose.arm1Pose+(error.arm1Pose/n);
            crtPose.arm2Pose=crtPose.arm2Pose+(error.arm2Pose/n);
            crtPose.tipPose=crtPose.tipPose+(error.tipPose/n);
            crtPose.swivelPose=crtPose.swivelPose+(error.swivelPose/n);

            setToPose();
            op.sleep(delay);
        }
    }

    // one function to set the robot to pose
    public void setToPose()
    {
        limitCheck();
        baseServo.setPosition(crtPose.basePose);
        arm1Servo.setPosition(crtPose.arm1Pose);
        arm2Servo.setPosition(crtPose.arm2Pose);
        tipServo.setPosition(crtPose.tipPose);
        swivelServo.setPosition(crtPose.swivelPose);

    }

    void printCrtPose()
    {
        op.telemetry.clearAll();
        // Display the current value
        op.telemetry.addData(">", "Press Stop to end test.");
        op.telemetry.addData("Current Base Position", "%5.2f", toPoseSample.crtPose.basePose);
        op.telemetry.addData("Current Arm 1 Position", "%5.2f", toPoseSample.crtPose.arm1Pose);
        op.telemetry.addData("Current Arm 2 Position", "%5.2f", toPoseSample.crtPose.arm2Pose);
        op.telemetry.addData("Current Tip Position", "%5.2f", toPoseSample.crtPose.tipPose);
        op.telemetry.addData("Current Swivel Position", "%5.2f", toPoseSample.crtPose.swivelPose);
        op.telemetry.addData("Radial Distance to Object", "%5.2f",interpol.R);
        op.telemetry.addData("DrivedLoc to Object", "%5.2f",interpol.DrivedLoc);
        op.telemetry.addData("ARM_FEAS to Object", "%5.2f",interpol.ARM_FEAS);
        op.telemetry.addData("X Full Length to Object", "%5.2f",interpol.xFullLen);

        op.telemetry.addLine(String.format("xLen: %5.2f, yLen1: %5.2f, yLen2:  %5.2f",interpol.xLen,interpol.yLen1,interpol.yLen2));
        op.telemetry.addLine(String.format("Servo moves %5.2f,  %5.2f,  %5.2f,  %5.2f",interpol.armPoses[0],interpol.armPoses[1],interpol.armPoses[2],interpol.armPoses[3]));

        op.telemetry.update();
    }
}
