/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef USING_OPENCV4
#include <opencv2/core.hpp>
#else
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>

#include "core/VioManager.h"
#include "utils/dataset_reader.h"

#include <fstream>
#include <iostream>
#include <algorithm>

using namespace ov_msckf;

VioManager* sys;

struct vel_acc_vector {
    double x, y, z;
};

struct imu_data {
    vel_acc_vector angular_velocity;
    vel_acc_vector linear_acceleration;
};

VioManagerOptions create_params()
{
    VioManagerOptions params;

    // Camera #0
    Eigen::Matrix<double, 8, 1> intrinsics_0;
    intrinsics_0 << 458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05;
    std::vector<double> matrix_TCtoI_0 = {0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
        0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
        0.0, 0.0, 0.0, 1.0};

    Eigen::Matrix4d T_CtoI_0;
    T_CtoI_0 << matrix_TCtoI_0.at(0), matrix_TCtoI_0.at(1), matrix_TCtoI_0.at(2), matrix_TCtoI_0.at(3),
        matrix_TCtoI_0.at(4), matrix_TCtoI_0.at(5), matrix_TCtoI_0.at(6), matrix_TCtoI_0.at(7),
        matrix_TCtoI_0.at(8), matrix_TCtoI_0.at(9), matrix_TCtoI_0.at(10), matrix_TCtoI_0.at(11),
        matrix_TCtoI_0.at(12), matrix_TCtoI_0.at(13), matrix_TCtoI_0.at(14), matrix_TCtoI_0.at(15);

    // Load these into our state
    Eigen::Matrix<double, 7, 1> extrinsics_0;
    extrinsics_0.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_0.block(0, 0, 3, 3).transpose());
    extrinsics_0.block(4, 0, 3, 1) = -T_CtoI_0.block(0, 0, 3, 3).transpose() * T_CtoI_0.block(0, 3, 3, 1);

    params.camera_fisheye.insert({0, false});
    params.camera_intrinsics.insert({0, intrinsics_0});
    params.camera_extrinsics.insert({0, extrinsics_0});

    params.camera_wh.insert({0, {752, 480}});

    // Camera #1
    Eigen::Matrix<double, 8, 1> intrinsics_1;
    intrinsics_1 << 457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, -0.00010473, -3.55590700e-05;
    std::vector<double> matrix_TCtoI_1 = {0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
        0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
        -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
        0.0, 0.0, 0.0, 1.0};

    Eigen::Matrix4d T_CtoI_1;
    T_CtoI_1 << matrix_TCtoI_1.at(0), matrix_TCtoI_1.at(1), matrix_TCtoI_1.at(2), matrix_TCtoI_1.at(3),
        matrix_TCtoI_1.at(4), matrix_TCtoI_1.at(5), matrix_TCtoI_1.at(6), matrix_TCtoI_1.at(7),
        matrix_TCtoI_1.at(8), matrix_TCtoI_1.at(9), matrix_TCtoI_1.at(10), matrix_TCtoI_1.at(11),
        matrix_TCtoI_1.at(12), matrix_TCtoI_1.at(13), matrix_TCtoI_1.at(14), matrix_TCtoI_1.at(15);

    // Load these into our state
    Eigen::Matrix<double, 7, 1> extrinsics_1;
    extrinsics_1.block(0, 0, 4, 1) = rot_2_quat(T_CtoI_1.block(0, 0, 3, 3).transpose());
    extrinsics_1.block(4, 0, 3, 1) = -T_CtoI_1.block(0, 0, 3, 3).transpose() * T_CtoI_1.block(0, 3, 3, 1);

    params.camera_fisheye.insert({1, false});
    params.camera_intrinsics.insert({1, intrinsics_1});
    params.camera_extrinsics.insert({1, extrinsics_1});

    params.camera_wh.insert({1, {752, 480}});

    // params.state_options.max_slam_features = 0;
    params.state_options.num_cameras = 2;
    params.init_window_time = 0.75;
    params.init_imu_thresh = 1.5;
    params.fast_threshold = 15;
    params.grid_x = 5;
    params.grid_y = 3;
    params.num_pts = 150;
    params.msckf_options.chi2_multipler = 1;
    params.knn_ratio = .7;

    params.state_options.imu_avg = true;
    params.state_options.do_fej = true;
    params.state_options.use_rk4_integration = true;
    params.use_stereo = true;
    params.state_options.do_calib_camera_pose = true;
    params.state_options.do_calib_camera_intrinsics = true;
    params.state_options.do_calib_camera_timeoffset = true;

    params.dt_slam_delay = 3.0;
    params.state_options.max_slam_features = 50;
    params.state_options.max_slam_in_update = 25;
    params.state_options.max_msckf_in_update = 999;

    params.use_aruco = false;

    params.state_options.feat_rep_slam = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");
    params.state_options.feat_rep_aruco = LandmarkRepresentation::from_string("ANCHORED_FULL_INVERSE_DEPTH");

#ifndef NDEBUG
    params.record_timing_information = true;
#endif /// NDEBUG

    return params;
}

void load_images(const string &file_name, unordered_map<double, string> &rgb_images,
                 vector<double> &timestamps) {
    ifstream file_in;
    file_in.open(file_name.c_str());

    // skip first line
    string s;
    getline(file_in, s);
    while (!file_in.eof()) {
        getline(file_in, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            double t;
            string rgb_image;
            ss >> t;
            timestamps.push_back(t);
            ss >> rgb_image;
            rgb_image.erase(std::remove(rgb_image.begin(), rgb_image.end(), ','), rgb_image.end());
            rgb_images[t] = rgb_image;
        }
    }
}

void load_imu_data(const string &file_name, unordered_map<double, imu_data> &imu_data_vals,
                   vector<double> &timestamps) {
    ifstream file_in;
    file_in.open(file_name.c_str());

    // skip first line
    string s;
    getline(file_in, s);
    while (!file_in.eof()) {
        getline(file_in, s);
        if (!s.empty()) {
            stringstream ss(s);
            string word;
            vector<double> line;
            while (getline(ss, word, ',')) {
                line.push_back(stod(word));
            }

            imu_data imu_vals;
            double t = line[0];
            timestamps.push_back(t);
            imu_vals.angular_velocity.x = line[1];
            imu_vals.angular_velocity.y = line[2];
            imu_vals.angular_velocity.z = line[3];
            imu_vals.linear_acceleration.x = line[4];
            imu_vals.linear_acceleration.y = line[5];
            imu_vals.linear_acceleration.z = line[6];
            imu_data_vals[t] = imu_vals;
        }
    }
}

// Main function
int main(int argc, char** argv) {

    if (argc != 6) {
        cerr << "Usage: ./run_serial_msckf path_to_cam0 path_to_cam1 path_to_imu0 path_to_cam0_images path_to_cam1_images" << endl;
        return 1;
    }

    unordered_map<double, string> cam0_images;
    unordered_map<double, string> cam1_images;
    unordered_map<double, imu_data> imu0_vals;
    vector<double> cam0_timestamps;
    vector<double> cam1_timestamps;
    vector<double> imu0_timestamps;
    string cam0_filename = string(argv[1]);
    string cam1_filename = string(argv[2]);
    string imu0_filename = string(argv[3]);
    string cam0_images_path = string(argv[4]);
    string cam1_images_path = string(argv[5]);

    load_images(cam0_filename, cam0_images, cam0_timestamps);
    load_images(cam1_filename, cam1_images, cam1_timestamps);
    load_imu_data(imu0_filename, imu0_vals, imu0_timestamps);

    cout << "cam0 images: " << cam0_images.size() << "  cam1 images: " << cam1_images.size() << "  imu0 data: " << imu0_vals.size() << endl;
    if (cam0_images.size() != cam1_images.size()) {
        cerr << "Mismatched number of cam0 and cam1 images!" << endl;
        return 1;
    }

    cout << "Finished Loading Data!!!!" << endl;

    // Create our VIO system
    auto params = create_params();
    sys = new VioManager(params);

    // Disabling OpenCV threading is faster on x86 desktop but slower on
    // jetson. Keeping this here for manual disabling.
    // cv::setNumThreads(0);

    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    max_cameras = 2; // max_cameras

    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
    double time = 0.0;
    double time_buffer = 0.0;

    // Load groundtruth if we have it
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

    // Loop through data files (camera and imu)
    unsigned num_images = 0;
    for (auto timem : imu0_timestamps) {
        // Handle IMU measurement
        if (imu0_vals.find(timem) != imu0_vals.end()) {
            // convert into correct format
            imu_data m = imu0_vals.at(timem);
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z;
            am << m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z;
            // send it to our VIO system
            sys->feed_measurement_imu(timem/1000000000.0, wm, am);
        }

        // Handle LEFT camera
        if (cam0_images.find(timem) != cam0_images.end()) {
            // Get the image
            img0 = cv::imread(cam0_images_path+ "/" +cam0_images.at(timem), cv::IMREAD_COLOR);
            cv::cvtColor(img0, img0, cv::COLOR_BGR2GRAY);
            if (img0.empty()) {
                cerr << endl << "Failed to load image at: "
                     << cam0_images_path << "/" << cam0_images.at(timem) << endl;
                return 1;
            }

            // Save to our temp variable
            has_left = true;
            time = timem/1000000000.0;
        }

        // Handle RIGHT camera
        if (cam1_images.find(timem) != cam1_images.end()) {
            // Get the image
            img1 = cv::imread(cam1_images_path+ "/" +cam1_images.at(timem), cv::IMREAD_COLOR);
            cv::cvtColor(img1, img1, cv::COLOR_BGR2GRAY);
            if (img1.empty()) {
                cerr << endl << "Failed to load image at: "
                     << cam1_images_path << "/" << cam1_images.at(timem) << endl;
                return 1;
            }

            has_right = true;
        }

        // Fill our buffer if we have not
        if(has_left && img0_buffer.rows == 0) {
            has_left = false;
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // Fill our buffer if we have not
        if(has_right && img1_buffer.rows == 0) {
            has_right = false;
            img1_buffer = img1.clone();
        }

        // If we are in monocular mode, then we should process the left if we have it
        if(max_cameras==1 && has_left) {
            // process once we have initialized with the GT
            Eigen::Matrix<double, 17, 1> imustate;
            if(!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
                //biases are pretty bad normally, so zero them
                //imustate.block(11,0,6,1).setZero();
                sys->initialize_with_gt(imustate);
            } else if(gt_states.empty() || sys->initialized()) {
                sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
            }
            // reset bools
            has_left = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
        }

        // If we are in stereo mode and have both left and right, then process
        if(max_cameras==2 && has_left && has_right) {
            // process once we have initialized with the GT
            Eigen::Matrix<double, 17, 1> imustate;
            if(!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(time_buffer,imustate,gt_states)) {
                //biases are pretty bad normally, so zero them
                //imustate.block(11,0,6,1).setZero();
                sys->initialize_with_gt(imustate);
            } else if(gt_states.empty() || sys->initialized()) {
                sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
            }
            // reset bools
            has_left = false;
            has_right = false;
            // move buffer forward
            time_buffer = time;
            img0_buffer = img0.clone();
            img1_buffer = img1.clone();

            num_images++;
        }

        //if (num_images == 500)
        //    break;
    }

    // Dump frame times
    /*{
        ofstream output_file;
        output_file.open("times.csv", std::ios::out);

        auto tracker_times = sys->get_tracker_times();
        auto filter_times = sys->get_filter_times();
        auto total_times = sys->get_total_times();

        output_file << "# Frame Tracker Filter Total\n";

        for (unsigned i = 0; i < total_times.size(); i++) {
            output_file << i << " " << tracker_times[i] << " " << filter_times[i] << " " << total_times[i] << endl;
        }

        output_file.close();
    }*/

    // Finally delete our system
    delete sys;

    // // Done!
    cout << "DONE!" << endl;
    return EXIT_SUCCESS;
}


















