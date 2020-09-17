#ifndef CONFIG_H
#define CONFIG_H

#include <vector>
#include <string>
#include <iostream>

namespace Config
{
    class Field
    {
    private:
        int RobotsCount = 3;
        double FieldLineWidth = 0.003;
        double FieldLength = 1.50;
        double FieldWidth = 1.300;
        double FieldRad = 0.200;
        double FieldFreeKick = 0.200;
        double FieldPenaltyWidth = 0.70;
        double FieldPenaltyDepth = 0.15;
        double FieldPenaltyPoint = 0.35;
        double FieldMargin = 0.3;
        double FieldRefereeMargin = 0.4;
        double WallThickness = 0.025;
        double GoalThickness = 0.025;
        double GoalDepth = 0.10;
        double GoalWidth = 0.40;
        double GoalHeight = 0.05;

    public:
        int getRobotsCount() { return this->RobotsCount; }
        double getFieldLineWidth() { return this->FieldLineWidth; }
        double getFieldLength() { return this->FieldLength; }
        double getFieldWidth() { return this->FieldWidth; }
        double getFieldRad() { return this->FieldRad; }
        double getFieldFreeKick() { return this->FieldFreeKick; }
        double getFieldPenaltyWidth() { return this->FieldPenaltyWidth; }
        double getFieldPenaltyDepth() { return this->FieldPenaltyDepth; }
        double getFieldPenaltyPoint() { return this->FieldPenaltyPoint; }
        double getFieldMargin() { return this->FieldMargin; }
        double getFieldRefereeMargin() { return this->FieldRefereeMargin; }
        double getWallThickness() { return this->WallThickness; }
        double getGoalThickness() { return this->GoalThickness; }
        double getGoalDepth() { return this->GoalDepth; }
        double getGoalWidth() { return this->GoalWidth; }
        double getGoalHeight() { return this->GoalHeight; }
    };

    class World
    {
    private:
        double BallRadius = 0.0215;
        int DesiredFPS = 60;
        double DeltaTime = 0.016;
        double Gravity = 9.8;
        bool ResetTurnOver = true;
        double BallMass = 0.043;
        double BallFriction = 0.05;
        int BallSlip = 1;
        double BallBounce = 0.5;
        double BallBounceVel = 0.1;
        double BallLinearDamp = 0.004;
        double BallAngularDamp = 0.004;

    public:
        double getBallRadius() { return this->BallRadius; }
        int getDesiredFPS() { return this->DesiredFPS; }
        double getDeltaTime() { return this->DeltaTime; }
        double getGravity() { return this->Gravity; }
        bool getResetTurnOver() { return this->ResetTurnOver; }
        double getBallMass() { return this->BallMass; }
        double getBallFriction() { return this->BallFriction; }
        int getBallSlip() { return this->BallSlip; }
        double getBallBounce() { return this->BallBounce; }
        double getBallBounceVel() { return this->BallBounceVel; }
        double getBallLinearDamp() { return this->BallLinearDamp; }
        double getBallAngularDamp() { return this->BallAngularDamp; }
    };

    class Communication
    {
    private:
        char *VisionMulticastAddr = "224.0.0.1";
        int VisionMulticastPort = 10020;
        int CommandListenPort = 20011;
        int BlueStatusSendPort = 30011;
        int YellowStatusSendPort = 30012;
        int sendDelay = 0;
        int sendGeometryEvery = 120;

    public:
        char *getVisionMulticastAddr() { return this->VisionMulticastAddr; }
        int getVisionMulticastPort() { return this->VisionMulticastPort; }
        int getCommandListenPort() { return this->CommandListenPort; }
        int getBlueStatusSendPort() { return this->BlueStatusSendPort; }
        int getYellowStatusSendPort() { return this->YellowStatusSendPort; }
        int getSendDelay() { return this->sendDelay; }
        int getSendGeometryEvery() { return this->sendGeometryEvery; }
    };

    class Noise
    {
    private:
        bool noise = false;
        int noiseDeviationX = 3;
        int noiseDeviationY = 3;
        int noiseDeviationAngle = 2;

    public:
        bool getNoise() { return this->noise; }
        int getNoiseDeviationX() { return this->noiseDeviationX; }
        int getNoiseDeviationY() { return this->noiseDeviationY; }
        int getNoiseDeviationAngle() { return this->noiseDeviationAngle; }
    };

    class Vanishing
    {
    public:
        bool vanishing = false;
        int blueTeamVanishing = 0;
        int yellowTeamVanishing = 0;
        int ballVanishing = 0;

    private:
        bool getVanishing() { return this->vanishing; };
        int getBlueTeamVanishing() { return this->blueTeamVanishing; };
        int getYellowTeamVanishing() { return this->yellowTeamVanishing; };
        int getBallVanishing() { return this->ballVanishing; };
    };

    class Robot
    {
    private:
        double CenterFromKicker = 0.05;
        double Radius = 0.0375;
        double Height = 0.02;
        double BottomHeight = 0.00;
        double KickerZValue = 0.005;
        double KickerThickness = 0.005;
        double KickerWidth = 0.08;
        double KickerHeight = 0.04;
        double WheelRadius = 0.02;
        double WheelThickness = 0.005;
        int Wheel1Angle = 90;
        int Wheel2Angle = 270;
        double BodyMass = 1.0;
        double WheelMass = 0.2;
        double KickerMass = 0.02;
        double KickerDampFactor = 0.2;
        double RollerTorqueFactor = 0.06;
        double RollerPerpendicularTorqueFactor = 0.005;
        double KickerFriction = 0.8;
        double WheelTangentFriction = 0.8;
        double WheelPerpendicularFriction = 1;
        double WheelMotorFMax = 0.2;
    public:
        double getCenterFromKicker(){return this->CenterFromKicker;}
        double getRadius(){return this->Radius;}
        double getHeight(){return this->Height;}
        double getBottomHeight(){return this->BottomHeight;}
        double getKickerZValue(){return this->KickerZValue;}
        double getKickerThickness(){return this->KickerThickness;}
        double getKickerWidth(){return this->KickerWidth;}
        double getKickerHeight(){return this->KickerHeight;}
        double getWheelRadius(){return this->WheelRadius;}
        double getWheelThickness(){return this->WheelThickness;}
        int getWheel1Angle(){return this->Wheel1Angle;}
        int getWheel2Angle(){return this->Wheel2Angle;}
        double getBodyMass(){return this->BodyMass;}
        double getWheelMass(){return this->WheelMass;}
        double getKickerMass(){return this->KickerMass;}
        double getKickerDampFactor(){return this->KickerDampFactor;}
        double getRollerTorqueFactor(){return this->RollerTorqueFactor;}
        double getRollerPerpendicularTorqueFactor(){return this->RollerPerpendicularTorqueFactor;}
        double getKickerFriction(){return this->KickerFriction;}
        double getWheelTangentFriction(){return this->WheelTangentFriction;}
        double getWheelPerpendicularFriction(){return this->WheelPerpendicularFriction;}
        double getWheelMotorFMax(){return this->WheelMotorFMax;}
    };

} // namespace Config

#endif