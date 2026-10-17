package org.firstinspires.ftc.teamcode.SampleDetector;

import com.acmerobotics.dashboard.config.Config;

@Config
public  final class Interpolate {

    public static double
            L1 = 120,
            L2 = 100,
            L3 = 140,
            baseHeight = 123,
            intAngle = 65,

            L11=236,//240,
            L12=365,//404,
            L21=211, //207,
            L22=347,//389,

            Sx1=150,//199,
            Sx2=147,//190,
            Sy1=211,//250,
            Sy2=183.5,//220,

            x1 = 66,//45,
            x2 = 254.5,//274,
            y1 = 51.5,//34,
            y2 = 182.5;//203;



    public static double CosAngle(double A,double B,double C){

        return (180- Math.toDegrees(Math.acos((A * A - B * B - C * C) / (2 * B * C))));

    }

    public static double CosLen(double Ang,double C,double B){

        return (Math.sqrt(B * B + C * C - (2 * B * C * Math.cos(Math.toRadians(Ang)))));
    }

    public  double
            Theta1 = CosAngle(L12,Sx1,L11),
            Theta2 = CosAngle(L22,Sx2,L21),
            angle_x12 = CosAngle(Sy1,L11,L21),

            dx = x2-x1,
            dy = y2-y1,

            TotalARM = L1+L2+L3,
            ARM_FEAS = Math.sqrt(L3*L3+(L1+L2)*(L1+L2)), //When L1&L2 inline and L3 perpendicular

            R = 0.0,/* BASE angle, ARM1 angle, ARM2 angle, CLAW angle */
            initialSwivel = 0,
            DrivedLoc,
            xFullLen,xLen,xMid,yLen1,yLen2,
            armPoses[]=new double [4];

    public void BaseAnglePose(double x,double y){
                x=x-x1;
                y=y-y1;

        for (int col = 0; col < armPoses.length; col++) {
            armPoses[col] = 0.0;
        }
                yLen1 = y/dy*Sx1;
                yLen2 = y/dy*Sx2;
         double
                dL11 = CosLen(Theta1,yLen1,L11),
                dL21 = CosLen(Theta2, yLen2,L21);
         double
                ang_dL11_dL21 = angle_x12-CosAngle(yLen1,dL11,L11)-CosAngle( yLen2,dL21,L21);

                xFullLen =CosLen(ang_dL11_dL21,dL11,dL21);

                xLen =xFullLen*x/dx;
                xMid =xFullLen/2;
         double
                xFullLenAngle = CosAngle(dL21,xFullLen,dL11);
         double
                Distance2Object = CosLen(xFullLenAngle,xLen,dL11);

        R = Distance2Object;
        DrivedLoc = Math.sqrt(R*R+baseHeight*baseHeight);
        armPoses[0] = toPoseSample.homePoseBase-((CosAngle(yLen1,dL11,L11)+CosAngle(xLen,Distance2Object,dL11)+intAngle)/300);
        initialSwivel = CosAngle(xMid,Distance2Object,dL11)-CosAngle(xLen,Distance2Object,dL11);
    }

    public boolean findSample(double x,double y){

         BaseAnglePose(x,y);

        if (DrivedLoc>(TotalARM-10)){
            return false;
        }

        double L3Angle=0;

        if(ARM_FEAS>DrivedLoc){
            L3Angle=90.0;
        }else
        {
            DrivedLoc=DrivedLoc-5;
            L3Angle=CosAngle(DrivedLoc,L3,(L1+L2));
            armPoses[1]=toPoseSample.arm1Horizontal+(CosAngle(L3,DrivedLoc,(L1+L2))-Math.toDegrees(Math.atan2(baseHeight,DrivedLoc)))/180;
            armPoses[2]=0.5;
            armPoses[3]=(L3Angle-90)/180;
            return true;
        }

        double L1Angle = CosAngle(Math.sqrt(L3*L3+L2*L2),DrivedLoc,L1)-Math.toDegrees(Math.atan2(baseHeight,DrivedLoc));
        double L2Angle= Math.toDegrees(Math.atan2(L3,L2))+CosAngle(DrivedLoc,L1,Math.sqrt(L3*L3+L2*L2));

        armPoses[1]=L1Angle/180+toPoseSample.arm1Horizontal;
        armPoses[2]=(281-L2Angle)/180;
        armPoses[3]=(L3Angle-90)/180;
        return true;
    }

    public double basePose(){
        return armPoses[0];
    }
    public double arm1Pose(){
        return armPoses[1];
    }
    public double arm2Pose(){
        return armPoses[2];
    }
    public double tipPose(){
        return armPoses[3];
    }
    public double swivelPose(){return initialSwivel;}
}