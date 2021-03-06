﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    public class Map
    {
        public int numMapSegments = 0;
        public double[,,] mapSegmentCorners;
        public double minX, maxX, minY, maxY;
        private double[] slopes;
        private double[] segmentSizes;
        private double[] intercepts;

        private double minWorkspaceX = -10;
        private double maxWorkspaceX =  10;
        private double minWorkspaceY = -10;
        private double maxWorkspaceY =  10;

        public Map()
        {

	        // This is hard coding at its worst. Just edit the file to put in
	        // segments of the environment your robot is working in. This is
	        // used both for visual display and for localization.

	        // ****************** Additional Student Code: Start ************
	
	        // Change hard code here to change map:

	        numMapSegments = 8;
            mapSegmentCorners = new double[numMapSegments, 2, 2];
            slopes = new double[numMapSegments];
            intercepts = new double[numMapSegments];
            segmentSizes = new double[numMapSegments];

            mapSegmentCorners[0, 0, 0] = 3.38 + 5.79 + 3.55 / 2;
	        mapSegmentCorners[0,0,1] = 2.794;
            mapSegmentCorners[0, 1, 0] = -3.38 - 5.79 - 3.55 / 2;
            mapSegmentCorners[0, 1, 1] = 2.794;

            mapSegmentCorners[1,0,0] = -3.55/2;
            mapSegmentCorners[1,0,1] = 0.0;
            mapSegmentCorners[1,1,0] = -3.55/2;
            mapSegmentCorners[1,1,1] = -2.74;

            mapSegmentCorners[2,0,0] = 3.55/2;
            mapSegmentCorners[2,0,1] = 0.0;
            mapSegmentCorners[2,1,0] = 3.55/2;
            mapSegmentCorners[2,1,1] = -2.74;

            mapSegmentCorners[3, 0, 0] = 3.55/2;
            mapSegmentCorners[3, 0, 1] = 0.0;
            mapSegmentCorners[3, 1, 0] = 3.55 / 2 + 5.79;
            mapSegmentCorners[3, 1, 1] = 0.0;

            mapSegmentCorners[4, 0, 0] = -3.55/2;
            mapSegmentCorners[4, 0, 1] = 0.0;
            mapSegmentCorners[4, 1, 0] = -3.55/2 - 5.79;
            mapSegmentCorners[4, 1, 1] = 0.0;

            mapSegmentCorners[5, 0, 0] = -3.55/2;
            mapSegmentCorners[5, 0, 1] = -2.74;
            mapSegmentCorners[5, 1, 0] = -3.55/2-3.05;
            mapSegmentCorners[5, 1, 1] = -2.74;

            mapSegmentCorners[6, 0, 0] = 3.55 / 2;
            mapSegmentCorners[6, 0, 1] = -2.74;
            mapSegmentCorners[6, 1, 0] = 3.55 / 2 + 3.05;
            mapSegmentCorners[6, 1, 1] = -2.74;

            mapSegmentCorners[7, 0, 0] = 5.03 / 2;
            mapSegmentCorners[7, 0, 1] = -2.74 - 2.31;
            mapSegmentCorners[7, 1, 0] = -5.03/2;
            mapSegmentCorners[7, 1, 1] = -2.74 - 2.31;
            // ****************** Additional Student Code: End   ************


	        // Set map parameters
	        // These will be useful in your future coding.
	        minX = 9999; minY = 9999; maxX=-9999; maxY=-9999;
	        for (int i=0; i< numMapSegments; i++){
		
		        // Set extreme values
                minX = Math.Min(minX, Math.Min(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                minY = Math.Min(minY, Math.Min(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
                maxX = Math.Max(maxX, Math.Max(mapSegmentCorners[i,0,0], mapSegmentCorners[i,1,0]));
                maxY = Math.Max(maxY, Math.Max(mapSegmentCorners[i,0,1], mapSegmentCorners[i,1,1]));
		
		        // Set wall segments to be horizontal
		        slopes[i] = (mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1])/(0.001+mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0]);
		        intercepts[i] = mapSegmentCorners[i,0,1] - slopes[i]*mapSegmentCorners[i,0,0];

		        // Set wall segment lengths
		        segmentSizes[i] = Math.Sqrt(Math.Pow(mapSegmentCorners[i,0,0]-mapSegmentCorners[i,1,0],2)+Math.Pow(mapSegmentCorners[i,0,1]-mapSegmentCorners[i,1,1],2));
	        }
        }


        // This function is used in your particle filter localization lab. Find 
        // the range measurement to a segment given the ROBOT POSITION (x, y) and 
        // SENSOR ORIENTATION (t)
        double GetWallDistance(double x, double y, double t, int segment){

	        // Set wall vars
	        double X1 = mapSegmentCorners[segment,0,0];
	        double Y1 = mapSegmentCorners[segment,0,1];
	        double X2 = mapSegmentCorners[segment,1,0];
	        double Y2 = mapSegmentCorners[segment,1,1];
	        double dist = 9999;

	        //Range t
	        if (t>Math.PI) t -= 2*Math.PI; else if (t<-Math.PI) t += 2*Math.PI;


	        // ****************** Additional Student Code: Start ***********

            double slopeR, slopeW, interceptR, interceptW;
            double xLineIntersect, yLineIntersect;
            double dotProduct, segmentLenSquare;

            slopeR = Math.Tan(t);
            slopeW = slopes[segment];

            interceptR = y - slopeR * x;
            interceptW = intercepts[segment];

            xLineIntersect = (interceptR - interceptW) / (slopeW - slopeR);
            yLineIntersect = slopeR * xLineIntersect + interceptR;

            segmentLenSquare = segmentSizes[segment];
            //dotProduct = (X1 - xLineIntersect) * (X2 - xLineIntersect) + (Y1 - yLineIntersect) * (Y2 - yLineIntersect);

            //if ((dotProduct > 0) && (segmentLenSquare > dotProduct))
            //{
            //    dist = (xLineIntersect - x) / Math.Cos(t);
            //}

            // Distance to wall.
            dist = Math.Sqrt(Math.Pow(x - xLineIntersect, 2) + Math.Pow(y - yLineIntersect, 2));

            // Need to make sure wall is in front of laser.
            // We do this by projecting dist from the robot according to t.
            // Next we find the distance between that point and the intersection point (should be small if valid).
            double xProj = x + dist * Math.Cos(t);
            double yProj = y + dist * Math.Sin(t);
            double projDist = Math.Sqrt(Math.Pow(xProj - xLineIntersect, 2) + 
                                        Math.Pow(yProj - yLineIntersect, 2));

            // Need to make sure wall extends to the intersection point.
            double projWallLen = Math.Sqrt(Math.Pow(xLineIntersect - (X1 + X2) / 2, 2) +
                                           Math.Pow(yLineIntersect - (Y1 + Y2) / 2, 2));

            // Dist is invalid if it's too long, the projected point is far from the predicted intersection, 
            // or the predicted intersection is not along the wall.
            if (dist > 9999 || Math.Abs(projDist) > Math.Abs(dist) || projWallLen > (segmentLenSquare / 2))
            {
                dist = 9999;
            }
	        // ****************** Additional Student Code: End   ************

	        return dist;
        }


        // This function is used in particle filter localization to find the
        // range to the closest wall segment, for a robot located
        // at position x, y with sensor with orientation t.

        public double GetClosestWallDistance(double x, double y, double t){

	        double minDist = 6.000; // This is in meters and the nominal max range of the laser range finder.

	        // ****************** Additional Student Code: Start ************

	        // Put code here that loops through segments, calling the
	        // function GetWallDistance.
            double nextDistance;

            // Keep track of which segment we got our distance from for debugging purposes.
            double segment = 0;

            for (int i = 0; i < numMapSegments; i++)
            {
                nextDistance = Math.Abs(GetWallDistance(x, y, t, i));
                if (nextDistance < minDist)
                {
                    segment = i;
                    minDist = nextDistance;
                }
            }
	        // ****************** Additional Student Code: End   ************

	        return minDist;
        }


        // This function is called from the motion planner. It is
        // used to check for collisions between an edge between
        // nodes n1 and n2, and a wall segment.
        // The function uses an iterative approach by moving along
        // the edge a "safe" distance, defined to be the shortest distance 
        // to the wall segment, until the end of the edge is reached or 
        // a collision occurs.

        bool CollisionFound(double n1x, double n1y, double n2x, double n2y, double tol){


	        // Check that within boundaries
	        if (n2x > maxWorkspaceX || n2x < minWorkspaceX || n2y > maxWorkspaceY || n2y < minWorkspaceY)
		        return true;


	        // Check for collision with walls
	        double theta = Math.Atan2(n2y-n1y, n2x-n1x);
	        double edgeSize = Math.Sqrt(Math.Pow(n2y-n1y, 2)+Math.Pow(n2x-n1x,2));
	        double sinTheta = Math.Sin(theta);
	        double cosTheta = Math.Cos(theta);

	        // Loop through segments
	        for (int segment=0; segment< numMapSegments; segment++) {

		        double distTravelledOnEdge = 0;
		        double ex = n1x, ey = n1y;
		        double distToSegment;
		        while (distTravelledOnEdge-tol < edgeSize){
			        distToSegment = GetWallDistance(ex, ey, segment, tol, n2x, n2y);
			        if (distToSegment-tol < 0.05)
				        return true;
			        ex += cosTheta*distToSegment;
			        ey += sinTheta*distToSegment;
			        distTravelledOnEdge +=distToSegment;
		        }

	        }
	        return false;
        }


        // This function will calculate the length of the perpendicular 
        // connecting point x,y to the wall segment. If the perpendicular
        // does not hit the segment, a large number is returned.

        double GetWallDistance(double x, double y, int segment, double tol, double n2x, double n2y){

	        // Set wall vars
	        double X1 = mapSegmentCorners[segment,0,0];
	        double Y1 = mapSegmentCorners[segment,0,1];
	        double X2 = mapSegmentCorners[segment,1,0];
	        double Y2 = mapSegmentCorners[segment,1,1];
	        double dist = 9999;

	        // Put code here to calculated dist.
	        // Calculate slope and intercept
	        double angleSegmentPerpendicular = Math.PI/2 + Math.Atan((Y2-Y1)/(0.000001+X2-X1));
	        double m = Math.Tan(angleSegmentPerpendicular);
	        double b = y - m*x;

	        // Get line intersection
	        double x_intersect = (b-intercepts[segment])/(slopes[segment]-m);
	        double y_intersect = m*x_intersect + b;

	        // Check to see if intersection LIES within segment
	        double dist_intersect_corner1 = Math.Sqrt(Math.Pow(x_intersect-X1,2) + Math.Pow(y_intersect-Y1,2));
	        double dist_intersect_corner2 = Math.Sqrt(Math.Pow(x_intersect-X2,2) + Math.Pow(y_intersect-Y2,2));
	        if (dist_intersect_corner1 <= (segmentSizes[segment]+tol) && dist_intersect_corner2 <= (segmentSizes[segment]+tol) ){
		        dist = Math.Sqrt(Math.Pow(x-x_intersect,2) + Math.Pow(y-y_intersect,2));
	        }
	
	        // Check for distance to corners (for case where no intersection with segment
	        double dist_point_corner1 = Math.Sqrt(Math.Pow(x-X1,2) + Math.Pow(y-Y1,2));
	        double dist_point_corner2 = Math.Sqrt(Math.Pow(x-X2,2) + Math.Pow(y-Y2,2));
	        dist = Math.Min(dist, dist_point_corner1); 
	        dist = Math.Min(dist, dist_point_corner2); 

	        return dist;
        }






    }
}
