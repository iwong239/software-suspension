#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

using namespace std;

float SoftStop(float height)
{
    float force = 0;
    if(height<=1)
    {
        force = 5;
    }
    else if(height<=2)
    {
        force = 10;
    }
    else if(height<=3)
    {
        force = 15;
    }
    else if(height<=4)
    {
        force = 20;
    }
    else if(height <= 5)
    {
        force = 25;
    }
    else
    {
        force = 30;
    }
    return force;
}

float MotorTorque(float height, float offload, float spoolcir, float motorradius, float k, float original, bool lswitch)
{
    float Spoolturn = ((height)/spoolcir)*2*M_PI;
    float springforce = original - (k* Spoolturn);  
    cout << original<< endl;
    float motorforce = offload - springforce;
    if(lswitch == true)
    {
        motorforce = motorforce - SoftStop(height - 6);
    }
    float motortorque = offload - springforce;
    return motortorque;
}



int main()
{
     // Moon gravity calculation
    float gravityinput = 1.6;
    float weight = 205;
    float mass = weight/9.81;
    float moonweight = mass*gravityinput;
    float offload = weight - moonweight;

    // Spool and Spring calculation
    float spooldiameter = 3; //in
    float SpoolCir = spooldiameter*M_PI;
    float MotorRadius = 2; //in
    float k = 17.62/(2*M_PI);//in*lb/radian
    float orispringforce= 170;//lb
    float dist1 = 0;
    float dist2 = 0;
    float disttemp = 0;
    float Equilibriumdist = 0;//in
    float initialzstop = Equilibriumdist+(2*12); //in
    float userheight = 10;
    bool lswitch = false;

    float dis[6] = {36, 36, 36, 36}; //in from equilibrium
    int lengtharray = sizeof(dis)/sizeof(dis[0]);
    
    bool stop = false;
    while(stop == false)
    {
        float total = 0;
        for(int i=0; i<lengtharray; i++)
        {
            total = total + dis[i];
        }
        float distance = total/lengtharray;
        float Torque = MotorTorque(distance, offload, SpoolCir, MotorRadius, k, orispringforce, lswitch);
        
        //stop = true;
        cout << "The final torque is: " << Torque << " lb*in^2/s^2" << endl;
    }
    

}