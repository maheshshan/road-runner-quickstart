package org.firstinspires.ftc.teamcode.FlagRobo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config

public final class toPose {

   public static class servoPoses
    {
        public double basePose, arm1Pose,arm2Pose,tipPose,swivelPose;
    }

    public Servo baseServo,arm1Servo,arm2Servo,clawServo,tipServo,swivelServo;

    private ElapsedTime runtime = new ElapsedTime();

    public static final double NumberOfSteps =40;
    private double n = NumberOfSteps;
    public static final long delayTime =40;
    private long delay= delayTime;

    public static  double pickPoseBase =0.20;
    public static  double pickPoseArm1 =0.3452;
    public static  double pickPoseArm2 =0.629;
    public static  double pickPoseTip =0.05;
    public static  double pickPoseSwivel =0.3;
    public static  double openClaw =0.45;
    public static  double closeClaw =0.02;
    public double clawPose =0.0;

    public static  double deliverPoseBase =0.8249;
    public static  double deliverPoseArm1 =0.3;
    public static  double deliverPoseArm2 =0.4831;
    public static  double deliverPoseTip =0.3;
    public static  double deliverPoseSwivel =0.5;

    public static  double homePoseBase =0.5;
    public static  double homePoseArm1 =0.95;
    public static  double homePoseArm2 =1;
    public static  double homePoseTip =0.1;
    public static  double homePoseSwivel =0.5;

    public static  double standPoseBase =0.2328;
    public static  double standPoseArm1 =0.7595;
    public static  double standPoseArm2 =0.7362;
    public static  double standPoseTip =0.0784;
    public static  double standPoseSwivel =0.5;

    public static  double showPoseBase =0.7743;
    public static  double showPoseArm1 =0.7595;
    public static  double showPoseArm2 =0.7362;
    public static  double showPoseTip =0.0784;
    public static  double showPoseSwivel =0.5;


 public static servoPoses crtPose = new servoPoses();
    public  servoPoses homePose = new servoPoses();
    public  servoPoses pickPose = new servoPoses();
    public  servoPoses deliverPose = new servoPoses();
    public  servoPoses standPose = new servoPoses();
    public  servoPoses showPose = new servoPoses();
    private servoPoses error = new servoPoses();

    private LinearOpMode op;

    public toPose(LinearOpMode p_op) {
        op = p_op;
    }
    public void initialize()
    {

    error.arm1Pose = 0;
    error.arm2Pose =0;
    error.basePose =0;
    error.tipPose =0;
    error.swivelPose=0;

    // Connect to servo (Assume Robot Left Hand)
    // Change the text in quotes to match any servo name on your robot.
    baseServo = op.hardwareMap.get(Servo.class, "Base");
    arm1Servo = op.hardwareMap.get(Servo.class, "Arm1");
    arm2Servo = op.hardwareMap.get(Servo.class, "Arm2");
    clawServo = op.hardwareMap.get(Servo.class, "Claw");
    tipServo = op.hardwareMap.get(Servo.class, "ClawTip");
    swivelServo = op.hardwareMap.get(Servo.class, "ClawSpin");


    homePose.arm1Pose = homePoseArm1;
    homePose.arm2Pose = homePoseArm2;
    homePose.basePose = homePoseBase;
    homePose.swivelPose = homePoseSwivel;
    homePose.tipPose = homePoseTip;

     crtPose.arm1Pose = homePoseArm1;
     crtPose.arm2Pose = homePoseArm2;
     crtPose.basePose = homePoseBase;
     crtPose.swivelPose = homePoseSwivel;
     crtPose.tipPose = homePoseTip;
     clawPose = openClaw;


    pickPose.arm1Pose = pickPoseArm1;
    pickPose.arm2Pose = pickPoseArm2;
    pickPose.basePose = pickPoseBase;
    pickPose.swivelPose = pickPoseSwivel;
    pickPose.tipPose = pickPoseTip;

    deliverPose.arm1Pose = deliverPoseArm1;
    deliverPose.arm2Pose = deliverPoseArm2;
    deliverPose.basePose = deliverPoseBase;
    deliverPose.tipPose = deliverPoseTip;
    deliverPose.swivelPose = deliverPoseSwivel;

    standPose.basePose = standPoseBase;
    standPose.arm1Pose = standPoseArm1;
    standPose.arm2Pose = standPoseArm2;
    standPose.tipPose = standPoseTip;
    standPose.swivelPose = standPoseSwivel;

    showPose.arm1Pose = showPoseArm1;
    showPose.arm2Pose = showPoseArm2;
    showPose.basePose = showPoseBase;
    showPose.tipPose = showPoseTip;
    showPose.swivelPose = showPoseSwivel;

     //clawServo.setPosition(closeClaw);
     toPose(homePose);
    }

    public void delivery()
    {
     home();
     show();
     toPose(deliverPose);
     op.sleep(1000);
     setOpenClaw();
    }

    public void stand() {
        toPose(standPose);

        for (int l = 0; l < 2; l++) {
            swivelServo.setPosition(showPoseSwivel + 0.4);
            op.sleep(500);
            swivelServo.setPosition(showPoseSwivel - 0.4);
            op.sleep(500);
            swivelServo.setPosition(showPoseSwivel);
        }


        for (int l = 0; l < 2; l++) {
            for (int j = 0; j < 11; j++) {
                tipServo.setPosition(showPoseTip + 0.02 * j);
                op.sleep(100);
            }
            for (int j = 0; j < 11; j++) {
                tipServo.setPosition(showPoseTip+0.2 - 0.02 * j);
                op.sleep(100);
            }
        }
    }

    public void show() {
     toPose(showPose);

     for (int l = 0; l < 2; l++) {
      swivelServo.setPosition(showPoseSwivel + 0.4);
      op.sleep(500);
      swivelServo.setPosition(showPoseSwivel - 0.4);
      op.sleep(500);
      swivelServo.setPosition(showPoseSwivel);
     }


     for (int l = 0; l < 2; l++) {
      for (int j = 0; j < 11; j++) {
       tipServo.setPosition(showPoseTip + 0.02 * j);
       op.sleep(100);
      }
      for (int j = 0; j < 11; j++) {
       tipServo.setPosition(showPoseTip+0.2 - 0.02 * j);
       op.sleep(100);
      }
     }
    }
    public void home()
    {
        toPose(homePose);
    }
    public void standPose(){
    toPose(homePose);
    }
    public void pick()
    {
     toPose(standPose);
     op.sleep(100);
     toPose(pickPose);
     op.sleep(1000);
     //setCloseClaw();
    }

    // one function to set the robot to pose
    private void setToPose()
    {
       baseServo.setPosition(crtPose.basePose);
       arm1Servo.setPosition(crtPose.arm1Pose);
       arm2Servo.setPosition(crtPose.arm2Pose);
       tipServo.setPosition(crtPose.tipPose);
       swivelServo.setPosition(crtPose.swivelPose);


     // Display the current value
     op.telemetry.addData("Current Base Position", "%5.2f", toPose.crtPose.basePose);
     op.telemetry.addData("Current Arm 1 Position", "%5.2f", toPose.crtPose.arm1Pose);
     op.telemetry.addData("Current Arm 2 Position", "%5.2f", toPose.crtPose.arm2Pose);
     op.telemetry.addData("Current Tip Position", "%5.2f", toPose.crtPose.tipPose);
     op.telemetry.addData("Current Swivel Position", "%5.2f", toPose.crtPose.swivelPose);
     //telemetry.addData("Current claw Position", "%5.2f", toPose.clawPose);


     op.telemetry.addData(">", "Press Stop to end test.");
     op.telemetry.update();

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
}
