//
//  icp.cpp
//  interactiveTable
//
//  Created by Matus Vojcik on 27/03/16.
//
//

#include "ofxICP.h"

ofxIcp::ofxIcp(){
    max_iterations = 10;
    closest_points_count = 20;
    error_threshold = 1.0;
}

void ofxIcp::compute(const vector<ofPoint> & initial, const vector<ofPoint> & target, vector<ofPoint> & output, ofMatrix4x4 & transformation, double & error, int & iterations){
    
    vector<cv::Point3d> initial_cv;
    vector<cv::Point3d> target_cv;
    
    toCV(initial, initial_cv);
    toCV(target, target_cv);
    
    cv::Mat transformation_matrix;
    
    vector<cv::Point3d> output_cv;
    compute(initial_cv, target_cv, output_cv, transformation_matrix, error, iterations);
    
    toOF(output_cv, output);
    
    transformation.set(transformation_matrix.ptr<double>());
}

void ofxIcp::compute(const vector<cv::Point3d> & input, const vector<cv::Point3d> & target, vector<cv::Point3d> & output, cv::Mat & transformation, double & error_, int & iterations){
    
    vector<cv::Mat> rotations;
    vector<cv::Mat> translations;
    
    vector<cv::Point3d> current = input;
    
    double previous_error, error = 0;
    int i;
    for(i = 0; i < max_iterations; i++){
        
        cv::Mat rotation_matrix;
        cv::Mat translation_vector;
        vector<cv::Point3d> current_transformed;
        
        icp_step(current, target, current_transformed, rotation_matrix, translation_vector, error);
        
        
        if(double_almost_equals(error, previous_error, 0.0001) && i != 0){
            break;
        }else{
            previous_error = error;
            current = current_transformed;
            rotations.push_back(rotation_matrix);
            translations.push_back(translation_vector);
        }
        
        if(error < error_threshold){
            break;
        }
    }
    
    compute_transformation(rotations, translations, transformation);
    output = current;
    
    error_ = error;
    iterations = i;
    
}

void ofxIcp::icp_step(const vector<cv::Point3d> & current, const vector<cv::Point3d> & target, vector<cv::Point3d> & output, cv::Mat & rotation, cv::Mat & translation, double & error){
   
    vector<cv::Point3d> closest_current;
    vector<cv::Point3d> closest_target;
    
    find_closest_points(closest_points_count, current, target, closest_current, closest_target);
    
    cv::Mat centroid_current;
    cv::Mat centroid_target;
    
    cv::reduce(cv::Mat(closest_current, false), centroid_current, 0, CV_REDUCE_AVG);
    cv::reduce(cv::Mat(closest_target, false), centroid_target, 0, CV_REDUCE_AVG);
    
    centroid_current = centroid_current.reshape(1);
    centroid_target = centroid_target.reshape(1);
    
    compute_rigid_transformation(closest_current, centroid_current, closest_target, centroid_target, rotation, translation);
    
    vector<cv::Point3d> transformed_closest_current;
    vector<cv::Point3d> transformed_current;
    
    transform(closest_current, rotation, translation, transformed_closest_current);
    transform(current, rotation, translation, transformed_current);
    
    compute_error(transformed_closest_current, closest_target, error);
    
    output = transformed_current;
}

struct closest_pair{
    cv::Point3d p1;
    cv::Point3d p2;
    float dist;
};

bool closest_pair_sort(const closest_pair & pair_1, const closest_pair & pair_2) { return pair_1.dist < pair_2.dist; }

template<class bidiiter>
bidiiter random_unique(bidiiter begin, bidiiter end, size_t num_random) {
    size_t left = std::distance(begin, end);
    while (num_random--) {
        bidiiter r = begin;
        std::advance(r, rand()%left);
        std::swap(*begin, *r);
        ++begin;
        --left;
    }
    return begin;
}

void ofxIcp::find_closest_points(const int & how_many, const vector<cv::Point3d> & input_1, const vector<cv::Point3d> & input_2, vector<cv::Point3d> & output_1, vector<cv::Point3d> & output_2){
    
    
    ofxNearestNeighbour3D tree;
    vector<ofPoint> of_Input_2;
    toOF(input_2, of_Input_2);
    tree.buildIndex(of_Input_2);
    
    vector<closest_pair> pairs;
    
    /*
     //RANDOM
     vector<cv::Point3d> input_1_tmp = input_1;
     random_unique(input_1_tmp.begin(), input_1_tmp.end(), how_many);
     for(int i = 0; i < how_many; i++){
     vector<float> distsSq;
     vector<NNIndex> indicesKNN;
     ofPoint p_tmp(input_1_tmp[i].x, input_1_tmp[i].y, input_1_tmp[i].z);
     tree.findNClosestPoints(p_tmp, 1, indicesKNN, distsSq);
     
     closest_pair pair;
     pair.p1 = input_1_tmp[i];
     pair.p2 = input_2[indicesKNN[0]];
     pair.dist = distsSq[0];
     
     pairs.push_back(pair);
     }*/
    
    //UNIFORM
    int increment = input_1.size()/how_many;
    for(int i = 0; i < input_1.size(); i += increment){
        vector<float> distsSq;
        vector<NNIndex> indicesKNN;
        ofPoint p_tmp(input_1[i].x, input_1[i].y, input_1[i].z);
        tree.findNClosestPoints(p_tmp, 1, indicesKNN, distsSq);
        
        closest_pair pair;
        pair.p1 = input_1[i];
        pair.p2 = input_2[indicesKNN[0]];
        pair.dist = distsSq[0];
        
        pairs.push_back(pair);
    }
    
    sort(pairs.begin(), pairs.end(), closest_pair_sort);
    
    
    for(int i = 0; i < pairs.size() * 0.8; i++){
        output_1.push_back(pairs[i].p1);
        output_2.push_back(pairs[i].p2);
    }
}

void ofxIcp::compute_rigid_transformation(const vector<cv::Point3d> & points_1, const cv::Mat & centroid_1, const vector<cv::Point3d> & points_2, const cv::Mat & centroid_2, cv::Mat & rotation, cv::Mat & translation){
    
    cv::Mat covariance_matrix;
    compute_covariance_matrix(points_1, centroid_1, points_2, centroid_2, covariance_matrix);
    
    compute_rotation_matrix(covariance_matrix, rotation);
    
    cv::Mat rot_centroid_1_mat = rotation * centroid_1.t();
    translation = centroid_2 - rot_centroid_1_mat.t();
}

void ofxIcp::compute_rotation_matrix(const cv::Mat & cov_matrix ,cv::Mat & output){
    cv::Mat w, u, vt;
    cv::SVD::compute(cov_matrix, w, u, vt);
    
    cv::Mat rotation_matrix = u*vt;
    
    double det = cv::determinant(rotation_matrix);
    
    if(det < 0){
        cv::Mat third_col = rotation_matrix.col(2);
        third_col *= -1;
    }
    
    output = rotation_matrix;
}

void ofxIcp::compute_covariance_matrix(const vector<cv::Point3d> & points_1, const cv::Mat & centroid_1, const vector<cv::Point3d> & points_2, const cv::Mat & centroid_2, cv::Mat & output){
    
    for(int i = 0; i < points_1.size(); i++){
        cv::Mat mat2 = centroid_1.t() - cv::Mat(points_1[i], false).reshape(1);
        cv::Mat mat1 = centroid_2.t() - cv::Mat(points_2[i], false).reshape(1);
        
        cv::Mat mat3 = mat1*mat2.t();
        
        if(i == 0){
            output = mat3;
        }else{
            output += mat3;
        }
    }
}

void ofxIcp::transform(const vector<cv::Point3d> & input, const cv::Mat & rotation_matrix, const cv::Mat & translation_vector, vector<cv::Point3d> & output){
    output.clear();
    
    cv::Mat current_trans_mat = rotation_matrix * cv::Mat(input, false).reshape(1).t();
    cv::transpose(current_trans_mat, current_trans_mat);
    
    for (int i = 0; i < current_trans_mat.rows; i++) {
        current_trans_mat.row(i) = current_trans_mat.row(i) + translation_vector;
        
        const double *data = current_trans_mat.row(i).ptr<double>(0);
        cv::Point3d p(data[0], data[1], data[2]);
        
        output.push_back(p);
    }
}

void ofxIcp::compute_error(const vector<cv::Point3d> & input_1, const vector<cv::Point3d> & input_2, double & output){
    
    double sum = 0;
    for(int i = 0; i < input_1.size(); i++){
        sum += cv::norm(input_1[i] - input_2[i]);
    }
    
    output = sum/input_1.size();
}

void ofxIcp::compute_transformation(const vector<cv::Mat> & rotations, const vector<cv::Mat> & translations, cv::Mat & transformation){
    
    for(int i = 0; i < rotations.size(); i++){
        cv::Mat transformation_tmp = (cv::Mat_<double>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
        cv::Mat rotation_tmp = transformation_tmp.colRange(0,3).rowRange(0,3);
        cv::Mat translation_tmp = transformation_tmp.colRange(3,4).rowRange(0,3);
        
        cv::Mat translation_t = translations[i].reshape(1).t();
        
        rotations[i].copyTo(rotation_tmp);
        translation_t.copyTo(translation_tmp);
        
        if(i == 0){
            transformation = transformation_tmp;
        }else{
            transformation *= transformation_tmp;
        }
    }
}

void ofxIcp::compute_transformation(const vector<cv::Point3d> & input, const vector<cv::Point3d> & target, cv::Mat & transformation){
    
    vector<cv::Point3d> transformed;
    cv::Mat rotation, translation;
    double error;
    
    vector<cv::Point3d> closest_current;
    vector<cv::Point3d> closest_target;
    
    int increment = input.size()/5;
    for(int i = 0; i < input.size(); i += increment){
        closest_current.push_back(input[i]);
        closest_target.push_back(target[i]);
    }
    
    cv::Mat centroid_current;
    cv::Mat centroid_target;
    
    cv::reduce(cv::Mat(closest_current, false), centroid_current, 0, CV_REDUCE_AVG);
    cv::reduce(cv::Mat(closest_target, false), centroid_target, 0, CV_REDUCE_AVG);
    
    centroid_current = centroid_current.reshape(1);
    centroid_target = centroid_target.reshape(1);
    
    compute_rigid_transformation(closest_current, centroid_current, closest_target, centroid_target, rotation, translation);
    
    vector<cv::Point3d> transformed_closest_current;
    vector<cv::Point3d> transformed_current;
    
    transform(closest_current, rotation, translation, transformed_closest_current);
    
    compute_error(transformed_closest_current, closest_target, error);
    
    transformation = (cv::Mat_<double>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    cv::Mat rotation_tmp = transformation.colRange(0,3).rowRange(0,3);
    cv::Mat translation_tmp = transformation.colRange(3,4).rowRange(0,3);
    
    cv::Mat translation_t = translation.reshape(1).t();
    
    rotation.copyTo(rotation_tmp);
    translation_t.copyTo(translation_tmp);
}

void ofxIcp::toCV(const vector<ofPoint> & input, vector<cv::Point3d> & output){
    output.clear();
    for(int i = 0; i < input.size(); i++){
        output.push_back(cv::Point3d(input[i].x,input[i].y,input[i].z));
    }
}

void ofxIcp::toOF(const vector<cv::Point3d> & input, vector<ofPoint> & output){
    output.clear();
    for(int i = 0; i < input.size(); i++){
        output.push_back(ofPoint(input[i].x,input[i].y,input[i].z));
    }
}

bool ofxIcp::double_almost_equals(const double & a, const double & b, const double & epsilon){
    return abs(a - b) < epsilon;
}