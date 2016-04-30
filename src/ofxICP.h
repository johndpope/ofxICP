//
//  icp.h
//  interactiveTable
//
//  Created by Matus Vojcik on 27/03/16.
//
//

#ifndef __interactiveTable__icp__
#define __interactiveTable__icp__

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxNearestNeighbour.h"

class ofxIcp{
public:
    
    ofxIcp();

    void setMaxIterations(const int & max_iterations_){max_iterations = max_iterations_;};
    
    void setClosestPointPairs(const int & closest_points_count_){closest_points_count = closest_points_count_;};
    
    void setErrorThreshold(const double & error_threshold_){error_threshold = error_threshold_;};
    
    void compute(const vector<ofPoint> & initial, const vector<ofPoint> & target, vector<ofPoint> & output, ofMatrix4x4 & transformation, double & error, int & iterations);
        
    void compute(const vector<cv::Point3d> & input, const vector<cv::Point3d> & target, vector<cv::Point3d> & output, cv::Mat & transformation, double & error_, int & iterations);

private:
    
    int max_iterations;
    int closest_points_count;
    double error_threshold;
    
    void icp_step(const vector<cv::Point3d> & current, const vector<cv::Point3d> & target, vector<cv::Point3d> & output, cv::Mat & rotation, cv::Mat & translation, double & error);

    void find_closest_points(const int & how_many, const vector<cv::Point3d> & input_1, const vector<cv::Point3d> & input_2,  vector<cv::Point3d> & output_1, vector<cv::Point3d> & output_2);

    void compute_rigid_transformation(const vector<cv::Point3d> & points_1, const cv::Mat & centroid_1, const vector<cv::Point3d> & points_2, const cv::Mat & centroid_2, cv::Mat & rotation, cv::Mat & translation);

    void compute_rotation_matrix(const cv::Mat & cov_matrix ,cv::Mat & output);

    void compute_covariance_matrix(const vector<cv::Point3d> & points_1, const cv::Mat & centroid_1, const vector<cv::Point3d> & points_2, const cv::Mat & centroid_2, cv::Mat & output);

    void transform(const vector<cv::Point3d> & input, const cv::Mat & rotation_matrix, const cv::Mat & translation_vector, vector<cv::Point3d> & output);

    void compute_error(const vector<cv::Point3d> & input_1, const vector<cv::Point3d> & input_2, double & output);
        
    void compute_transformation(const vector<cv::Mat> & rotations, const vector<cv::Mat> & translations, cv::Mat & transformation);
    
    void compute_transformation(const vector<cv::Point3d> & input, const vector<cv::Point3d> & target, cv::Mat & transformation);

    void toCV(const vector<ofPoint> & input, vector<cv::Point3d> & output);
        
    void toOF(const vector<cv::Point3d> & input, vector<ofPoint> & output);
    
    bool double_almost_equals(const double & a, const double & b, const double & epsilon);
};

#endif /* defined(__interactiveTable__icp__) */
