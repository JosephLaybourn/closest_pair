#include "closestpair.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <utility>
#include <limits>

/* Following two functions are needed for library function qsort().  
Refer: http://www.cplusplus.com/reference/clibrary/cstdlib/qsort/ */
  
// Needed to sort array of points  
// according to X coordinate  
int compareX(const void* a, const void* b)  
{  
		const Point *point1 = reinterpret_cast<const Point*>(a);
		const Point *point2 = reinterpret_cast<const Point*>(b);
    return (point1->x - point2->x);  
}  
  
// Needed to sort array of points according to Y coordinate  
int compareY(const void* a, const void* b)  
{  
		const Point *point1 = reinterpret_cast<const Point*>(a);
		const Point *point2 = reinterpret_cast<const Point*>(b);
    return (point1->y - point2->y);  
}  
  
// A utility function to find the  
// distance between two points  
float dist(Point p1, Point p2)  
{  
    return sqrt( (p1.x - p2.x)*(p1.x - p2.x) +  
                (p1.y - p2.y)*(p1.y - p2.y)  
            );  
}  
  
// A Brute Force method to return the  
// smallest distance between two points  
// in P[] of size n  
float bruteForce(std::vector<Point>& P)  
{  
    float min = std::numeric_limits<float>::max();  
    for (int i = 0; i < int(P.size()); ++i)  
        for (int j = i+1; j < int(P.size()); ++j)  
            if (dist(P[i], P[j]) < min)  
                min = dist(P[i], P[j]);  
    return min;  
}  
  
// A utility function to find  
// minimum of two float values  
float min(float x, float y)  
{  
    return (x < y)? x : y;  
}  
  
  
// A utility function to find the  
// distance beween the closest points of  
// strip of given size. All points in  
// strip[] are sorted accordint to  
// y coordinate. They all have an upper 
// bound on minimum distance as d.  
// Note that this method seems to be  
// a O(n^2) method, but it's a O(n)  
// method as the inner loop runs at most 6 times  
float stripClosest(std::vector<Point>& strip, int size, float d)  
{  
    float min = d; // Initialize the minimum distance as d  
  
    qsort(strip.data(), size, sizeof(Point), compareY);  
  
    // Pick all points one by one and try the next points till the difference  
    // between y coordinates is smaller than d.  
    // This is a proven fact that this loop runs at most 6 times  
    for (int i = 0; i < size; ++i)  
        for (int j = i+1; j < size && (strip[j].y - strip[i].y) < min; ++j)  
            if (dist(strip[i],strip[j]) < min)  
                min = dist(strip[i], strip[j]);  
  
    return min;  
}  
  
// A recursive function to find the  
// smallest distance. The array P contains  
// all points sorted according to x coordinate  
float closestUtil(std::vector<Point>& P)  
{  
    // If there are 2 or 3 points, then use brute force  
    if (P.size() <= 3)  
        return bruteForce(P);  
  
    // Find the middle point  
    int mid = P.size()/2;  
    Point midPoint = P[mid];  
  
    // Consider the vertical line passing  
    // through the middle point calculate  
    // the smallest distance dl on left  
    // of middle point and dr on right side  
		std::vector<Point> left(P.data(), P.data() + mid);
		std::vector<Point> right(P.data() + mid, P.data() + (mid * 2));
    float dl = closestUtil(left);  
    float dr = closestUtil(right);  
  
    // Find the smaller of two distances  
    float d = min(dl, dr);  
  
    // Build an array strip[] that contains  
    // points close (closer than d)  
    // to the line passing through the middle point  
    std::vector<Point> strip(P.size());
    int j = 0;  
    for (int i = 0; i < int(P.size()); i++)  
        if (abs(P[i].x - midPoint.x) < d)  
            strip[j] = P[i], j++;  
  
    // Find the closest points in strip.  
    // Return the minimum of d and closest  
    // distance is strip[]  
    return min(d, stripClosest(strip, j, d) );  
}  
  
// The main function that finds the smallest distance  
// This method mainly uses closestUtil()  
float closest(std::vector<Point>& P)  
{  
    qsort(P.data(), P.size(), sizeof(Point), compareX);  
  
    // Use recursive function closestUtil() 
    // to find the smallest distance  
    return closestUtil(P);  
} 


std::ostream& operator<< (std::ostream& os, Point const& p) {
	os << "(" << p.x << " , " << p.y << ") ";
	return os;
}

std::istream& operator>> (std::istream& os, Point & p) {
	os >> p.x >> p.y;
	return os;
}

////////////////////////////////////////////////////////////////////////////////
float closestPair_aux (std::vector<Point>& points);

////////////////////////////////////////////////////////////////////////////////
float closestPair ( std::vector<Point> const& points ) {
	int size = points.size();

	if (size==0) throw "zero size subset";
	if (size==1) return std::numeric_limits<float>::max();

	std::vector<Point> copy(points);
	return closestPair_aux(copy);
}

////////////////////////////////////////////////////////////////////////////////
float closestPair_aux (std::vector<Point>& points) {
	int size = points.size();

	if (size==0) throw "zero size subset";
	if (size==1) return std::numeric_limits<float>::max();

	//.................

	return closest(points);
}

unsigned findMax(std::vector<int>& input, unsigned startingIndex)
{
    if(input.size() == 1)
        return startingIndex;
    
    std::vector<int> lowerHalf(input.data(), input.data() + (input.size() / 2));
    std::vector<int> upperHalf(input.data() + (input.size() / 2) + 1, input.data() + input.size());

    unsigned lowerBound = findMax(lowerHalf, startingIndex);
    unsigned upperBound = findMax(upperHalf, startingIndex + (input.size() / 2));

    if(lowerBound >= upperBound)
        return startingIndex;
    else
        return startingIndex + (input.size() / 2);
}

void sortNegativePositive(std::vector<float>& input)
{
    for(unsigned i = 0; i < input.size(); ++i)
    {
        if(input[i] > 0)
        {
            for(unsigned j = i; j < input.size(); ++j)
            {
                if(input[j] <= 0)
                {
                    float temp = input[j];
                    input[j] = input[i];
                    input[i] = temp;

                    break;
                }
            }
        }
    }
}

std::vector<std::pair<unsigned, unsigned>> assignHardware(std::vector<float> nutWidths, std::vector<float> boltWidths)
{
    std::vector<std::pair<unsigned, unsigned>> pairedIndicies;

    for(unsigned i = 0; i < nutWidths.size(); ++i)
    {
        for(unsigned j = 0; j < boltWidths.size(); ++j)
        {
            if(nutWidths[i] == boltWidths[j])
            {
                pairedIndicies[i] = std::pair<unsigned, unsigned>(i, j);
                boltWidths[j] = boltWidths.back();
                boltWidths.pop_back();

                break;
            }
        }
    }

    return pairedIndicies;
}