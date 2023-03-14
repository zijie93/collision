#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <float.h>
#include <set>
using namespace std;

vector<double> insecLine (vector<vector<double>> line1, vector<vector<double>> line2) {
     double x1=line1[0][0], y1=line1[0][1];
     double x2=line1[1][0], y2=line1[1][1];

     double x3=line2[0][0], y3=line2[0][1];
     double x4=line2[1][0], y4=line2[1][1];

     double det= (x3 - x4) * (y1 - y2) - (x1 - x2) * (y3 - y4);

     /*not need to add codes to check if two lines overlap for det=0, see the figure*/

     vector<double>res;
     if (det!=0) {
        double A1=x1-x2, B1=y1-y2;
        double A2=x3-x4, B2=y3-y4;
        double C1=x1*y2-y1*x2, C2=x3*y4-y3*x4;
        double x0=(A1*C2-A2*C1)/det; // get the intersection point
        double y0=(B1*C2-B2*C1)/det;
       
        if (x0<=max(x1,x2) && x0>=min(x1,x2) && x0<=max(x3,x4) && x0>=min(x3,x4) // check if the point is within the line segment
            && y0<=max(y1,y2) && y0>=min(y1,y2) && y0<=max(y3,y4) && y0>min(y3,y4)) { //do not count low end point of the line 2
            res={x0,y0};
        }
     }
     return res;
}

bool pointInPoly (vector<double> point, vector<vector<double>> poly) {
    int num=poly.size();
    double mX=DBL_MIN;
    for (int i=0; i<num; i++) {
        if (point[0]==poly[i][0] && point[1]==poly[i][1]) return true; // check if the point is one of the convex of the poly
        mX=max(mX, poly[i][0]); 
    }
    mX+=10; // get the right end of the checking line starting at the point
    vector<vector<double>> cline={{point[0], point[1]}, {mX, point[1]}}; // the checking line

    int count=0;
    for (int i=0; i<num; i++) {
        double x1=poly[i][0], y1=poly[i][1];
        double x2, y2;
        if (i<num-1) {
            x2=poly[i+1][0], y2=poly[i+1][1];
        } else {
            x2=poly[0][0], y2=poly[0][1];
        }
        vector<vector<double>>line={{x1,y1},{x2,y2}};
        
        if (insecLine(cline, line).size()>0) count++;
    }

    if (count%2==1) return true;
    else return false;
}

void pointAdd (vector<double> &point, vector<vector<double>> & list) {
    bool flag=false;
    for (int i=0; i<list.size(); i++) {
        if (point[0]==list[i][0] || point[1]==list[i][1]) {
            flag=true;
            break;
        } 
    }
    if (!flag) list.push_back(point);
}


bool static sortType (const vector<double> &v1, const vector<double> &v2) {
        return v1[2]<v2[2]; // accending order
}
vector<vector<double>> sortPoints (set<vector<double>> & set1) {
    vector<vector<double>>list; // sort the points by anti-clock order using arctan from their center point
    if (set1.size()==0) return list;
    double avX=0, avY=0;
    for (vector<double> it:set1) {
        list.push_back(it); 
        avX+=it[0];
        avY+=it[1];
    } 
    avX/=set1.size();
    avY/=set1.size();
    for (int i=0; i<list.size(); i++) {
        double temp=atan2((list[i][1]-avY), (list[i][0]-avX));
        list[i].push_back(temp);
    }
    sort(list.begin(), list.end(), sortType); 
    return list;
}

double insecArea(vector<vector<double>> &list) {
    if (list.size()<=2) return 0;
    double area=0, xo=list[0][0], yo=list[0][1];
    int i=1;
    while (i+1<list.size()) {
          double x1=list[i][0], y1=list[i][1];
          double x2=list[i+1][0], y2=list[i+1][1];
          double triArea=0.5*abs(xo*(y1-y2)+x1*(y2-yo)+x2*(yo-y1));
          area+=triArea;
          i++;
    }
    return area;
}

int main () {
    vector<vector<double>> poly1;
    vector<vector<double>> poly2;
    poly1={{3,0},{5,5},{3,10},{-3,10},{-5,5},{-3,0}};
    poly2={{0,5},{5,0},{10,5},{5,10}};

    vector<vector<double>> pointList;
    set<vector<double>>set1;

    for (int i=0; i<poly1.size(); i++) { // check if the convexs of poly1 are inside the poly2
        //if (pointInPoly(poly1[i], poly2)) pointAdd(poly1[i], pointList);  
        cout<<"point "<<poly1[i][0]<<" "<<poly1[i][1]<<endl;
        if (pointInPoly(poly1[i], poly2)) { 
            set1.insert(poly1[i]);
        }
    }

    for (int i=0; i<poly2.size(); i++) { // check if the convexs of poly2 are inside the poly1
        //if (pointInPoly(poly2[i], poly1)) pointAdd(poly2[i], pointList);
        if (pointInPoly(poly2[i], poly1)) {
            set1.insert(poly2[i]);
        }
    }
    
    vector<vector<double>> line1;
    vector<vector<double>> line2;
    for (int i=0; i<poly1.size(); i++) {  // check if the lines in poly1 intersec with lines in poly2 
        if (i<poly1.size()-1) line1={{poly1[i][0], poly1[i][1]}, {poly1[i+1][0], poly1[i+1][1]}};
        else line1={{poly1[i][0], poly1[i][1]}, {poly1[0][0], poly1[0][1]}};
        for (int j=0; j<poly2.size(); j++) {
            if (j<poly2.size()-1) line2={{poly2[j][0], poly2[j][1]}, {poly2[j+1][0], poly2[j+1][1]}};
            else line2={{poly2[j][0], poly2[j][1]}, {poly2[0][0], poly2[0][1]}};
            vector<double>res=insecLine(line1,line2);
            if (res.size()>0) {
                set1.insert(res);
            }
        }
    }

    vector<vector<double>> list=sortPoints(set1); // sort the points in anti-clock order in [-pi, pi] using atan2
  
    double area=insecArea(list); // calculate the intersection area of the two polygons
    cout<<"the intersection area of two polygons is "<<area<<endl;
    return 0;
}

    //vector<double>point={2,6};
    //if (pointInPoly(point, poly2)) cout<<"inside "<<endl;
    //else cout<<"outside "<<endl;
    //set1.insert(poly1[0]);

    //cout<<"the x2 and y2 is: "<<x2<<" "<<y2<<endl; 
    //    cout<<line[0][0]<<endl;
    //    cout<<line[0][1]<<endl;
    //    cout<<line[1][0]<<endl;
    //    cout<<line[1][1]<<endl;

    //cout<<"point "<<poly1[i][0]<<" "<<poly1[i][1]<<" is inside"<<endl;
    //cout<<"the set's size is: "<<set1.size()<<endl;
    

