
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <cmath>  
#include <cstdlib> //This header defines several general purpose functions, including dynamic memory management, random number generation, communication with the environment, integer arithmetics, searching, sorting and converting, such as atoi, stoi.
#include <ctime> 
using namespace std;

vector<vector<double>> getPoints() {  
    int choice;
    vector<vector<double>> waypoints;
    cout<<"enter 1 for using default waypoints; 2 for customization: "; cin>>choice;
    if (choice==1) {
        waypoints={
            {10000,10000}, 
            {18000,8000}, 
            {30000,12000}, 
            {40000,22000}, 
            {46000,37000}, 
            {38000,41000}, 
            {22000,30000}, 
            {11000,20000}, 
            {-8000,8000}
        }; // in x-y axis, unit: m.
    }else { 
        int num;
        double x, y;
        cout<<"please enter the number of waypoints "; cin>>num;
        for (int i=0; i<num; i++) {
            cout<<"please enter the "<< i+1 <<"th waypoint ";
            cout<<"please enter the x coordinate: "; cin>>x;
            cout<<"please enter the y coordinate: "; cin>>y;
            waypoints.push_back({x,y});
        }
    }      
    return waypoints;
}

vector<vector<double>> getWind(int n) {
    // wind speed and direction near the paths;
    int choice;
    vector<vector<double>>wind(n, vector<double>(2,0));  //vector<vector<double>> wind;
    cout<<"enter 1 for using default wind; 2 for randomly generation: "; cin>>choice;
    if (choice==1) {
        wind={
            {15,-45}, 
            {0,0}, 
            {0,0}, 
            {9,90}, 
            {0,0}, 
            {2,100}, 
            {15,105}, 
            {15,120}, 
            {0,0}
        }; 
        for (int i=0; i<wind.size(); i++) wind[i][1]=wind[i][1]*M_PI/180;
    }else {
        srand(time(0));
        int wind_max=22; // maximum wind speed;
        for (int i=0; i<n; i++) {
            wind[i][0]=(double)(rand()%wind_max); // wind speed
            wind[i][1]=(double)(-180+rand()%360)/180*M_PI; int wind_di=rand()%180; // rand()%22;
        }    
    }
    return wind;
}

vector<double> getUAV () {
    vector<double> uavData(4,0);    
    double g=9.8, speed=30, bank=30; // bank angle;
    double inEnergy=200, p=100; // initial energy and power rate;
    int choice;
    cout<<"enter 1 for using default maximum turning rate; 2 for customization: "; cin>>choice;
    if (choice==1) {
        // default maximum bank angle set to 30 degree
    }else {
        cout<<"enter the bank angle between 15 and 30 degree: "; cin>>bank;
    }
    double R=speed*speed/(g*tan(bank/180*M_PI)); // turning radius

    uavData[0]=speed; 
    uavData[1]=R;
    uavData[2]=inEnergy;
    uavData[3]=p;
    return uavData;
}

vector<double> uavDesired(double path_d, double wind_d, double va, double vw){
        double uav_dr;
        double theta=path_d-wind_d; // angle between required path and the wind
        if (theta<-M_PI) theta=theta+2*M_PI;
        else if (theta>M_PI) theta=theta-2*M_PI;
        /* for uav: vg(ground speed vector)=va(airspeed vector)+wind vector, implement triangle relation, here is:*/
        double vg=vw*cos(theta)+sqrt(va*va-vw*vw*sin(theta)*sin(theta)); // ground speed
        double gama=acos((va*va-vw*vw)/(vg*va)+vw/va*cos(theta));  // angle between required path and the uav air velocity
       
        if (theta>=0)
            uav_dr=gama+path_d; // uav desired direction
        else
            uav_dr=-gama+path_d; // uav desired direction

        if (uav_dr<-M_PI) uav_dr=uav_dr+2*M_PI;
        else if (uav_dr>M_PI) uav_dr=uav_dr-2*M_PI;

        return {uav_dr,vg};
}

double PID (double uav_dr, double uav_dc, double va, double R, double dt) {
        double w, k=1;
        double diff=uav_dr-uav_dc;
        if (diff<-M_PI) diff=diff+2*M_PI;
        else if (diff>M_PI) diff=diff-2*M_PI;
        w=k*diff;// turning angle
        if (w>va/R) {
            w=va/R;
        }else if (w<-va/R) {
            w=-va/R;
        }
        uav_dc=uav_dc+w*dt; // uav airspeed direction;
        if (uav_dc<-M_PI) uav_dc=uav_dc+2*M_PI;
        if (uav_dc>M_PI) uav_dc=uav_dc-2*M_PI;  
        return uav_dc;     
}

void printInf(double i, double uav_dr, double t, double Xcur, double Ycur, double uav_dc, double res, double inE, int bftOrAft) {
    if (!bftOrAft) {
        cout<<"\n";
        cout<<"To waypoint "<<i+1<<": "<<endl;
        cout<<"UAV should fly around "<<uav_dr*180/M_PI<<" degrees, considering of the wind. "<<endl;
    }else{
        cout<<"time flight is: "<<t<<endl;
        cout<<"uav current x and y position is: "<<Xcur<<", "<<Ycur<<endl;
        cout<<"uav current airspeed direction is: "<<uav_dc*180/M_PI<<endl;
        if (res<=0.2*inE) cout<<"warning: less than 20% battery! "<<endl;
        cout<<"the res of battery is: "<<res<<endl;
    }
}

double uavEnergy (vector<vector<double>>& waypoints, vector<vector<double>> &wind, vector<double> &uavData) {
    double va=uavData[0], R=uavData[1], inE=uavData[2], p=uavData[3]; // uav speed, R, initial battery, power rate;
    double res=inE;
    double Xcur=0, Ycur=0; // uav start point;
    double path_d; // path_d: path direction between waypoints and it is also the uav ground velocity desired direction
    double wind_d, uav_dr, uav_dc, w;//wind_d: wind direction; uav_dr: uav air velocity desired direction; uav_dc: uav current air velocity direction
    double theta, gama; // gama: angle between required path and the uav air velocity; theta: angle between required path and the wind 
    double distance, vw,  vg;  // distance: distance between uav and the next waypoint; vw: wind speed; vg: uav ground speed; 
    double t=0, dt=0.1, T=100000; // t: time; dt: time step; T: maximum boundary and has no physical meaning
    double k=1; // parameter of PID controller;
    double error=10; // bias of the distance to waypoints.

    int n=waypoints.size();
    for (int i=0; i<n; i++) {
        vw=wind[i][0]; // wind speed
        wind_d=wind[i][1]; // wind direction

        // calculate the approximate direction for the uav
        path_d=atan2(waypoints[i][1]-Ycur, waypoints[i][0]-Xcur); // path direction
        vector<double> uav_tmp=uavDesired(path_d, wind_d, va, vw);
        uav_dr=uav_tmp[0]; // uav_dr: uav air velocity desired direction; 
        vg=uav_tmp[1]; //vg: uav ground speed; 
        
        printInf(i, uav_dr, t, Xcur, Ycur, uav_dc, res, inE, 0);
        
        if (i==0) {
            uav_dc=uav_dr; // uav try to reach waypoint 1 from initial point;
            while(t<T) {
                Xcur=Xcur+vg*cos(path_d)*dt; // x-y position is relative to ground speed
                Ycur=Ycur+vg*sin(path_d)*dt;
                distance=sqrt((waypoints[i][1]-Ycur)*(waypoints[i][1]-Ycur)+(waypoints[i][0]-Xcur)*(waypoints[i][0]-Xcur));
                t+=dt;
                if (distance<=10) {
                    break;
                }
            }
        }else{
            double vgc_x, vgc_y; // uav ground speed in x, y with time;
            while(t<T) {
                    
                // calculate the local finer controller. UAV direction should be updated time by time;
                path_d=atan2(waypoints[i][1]-Ycur, waypoints[i][0]-Xcur); // path direction
                theta=path_d-wind_d; // angle between required path and the wind
                vector<double> uav_tmp=uavDesired(path_d, wind_d, va, vw);
                uav_dr=uav_tmp[0]; // uav_dr: uav air velocity desired direction; 
                vg=uav_tmp[1]; //vg: uav ground speed; 
                
                // PID controller
                uav_dc=PID(uav_dr, uav_dc, va, R, dt);
                vgc_x=va*cos(uav_dc)+vw*cos(wind_d); // uav ground speed in x axis;
                vgc_y=va*sin(uav_dc)+vw*sin(wind_d); // uav ground speed in y axis;
                Xcur=Xcur+vgc_x*dt; 
                Ycur=Ycur+vgc_y*dt; 
                distance=sqrt((waypoints[i][1]-Ycur)*(waypoints[i][1]-Ycur)+(waypoints[i][0]-Xcur)*(waypoints[i][0]-Xcur));
                t+=dt;
                if (distance<=10) break; // search for next waypoint;
            }
        }

        res=inE-p*t/3600;
        printInf(i, uav_dr, t, Xcur, Ycur, uav_dc, res, inE, 1);
        
    }
    return res;
}
int main()
{
    // get waypoints 
    vector<vector<double>> waypoints=getPoints();
    
    // get wind information
    vector<vector<double>> wind=getWind(waypoints.size());

    // get uav information
    vector<double>uavData=getUAV();

    // get the res battery (energy)
    double res=uavEnergy(waypoints, wind, uavData);
       
    return 0;
}