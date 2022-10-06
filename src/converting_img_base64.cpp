#include <fstream>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "converting_base64/base64.h"

int main(int argc, char** argv) {
    bool all_tests_passed = true;
    std::string line;

    std::string filepath = ros::package::getPath("encode_n_decode_test");

    std::ifstream input(filepath + "/maps/mymap.pgm", std::ios::in | std::ios::binary);
    std::ofstream output(filepath + "/output/output.txt");
    std::ofstream output_pgm(filepath + "/maps/decoded_map.pgm");

    if(input.is_open()) {
        while(getline(input, line)) {
            
            std::string encoded_str = base64_encode(reinterpret_cast<const unsigned char*>(line.c_str()), line.length());
            std::string decoded_str = base64_decode(encoded_str, true);

            if (decoded_str != line) {
                std::cout << "decoded_str is not the same as input line" << std::endl;
                all_tests_passed = false;
            }

            output << encoded_str + "\n";  // add linebreak
            output_pgm << decoded_str + "\n"; // add linebreak  // 복원했을 때 이미지가 다시 제대로 표현 안되는 문제, line 들이 엔터가 안되는 현상
        }
        input.close();
    }

    if (all_tests_passed) {
        ROS_INFO("test PASSED");
    } else {
        ROS_ERROR("test FAILED");
    }

    ////// test /////////
    // const std::string orig =
    // "René Nyffenegger\n"
    // "http://www.renenyffenegger.ch\n"
    // "passion for data\n";

    // std::string encoded_str = base64_encode(reinterpret_cast<const unsigned char*>(orig.c_str()), orig.length());
    // std::string decoded = base64_decode(encoded_str);

    // std::cout << encoded_str << std::endl;
    // if (encoded_str != "UmVuw6kgTnlmZmVuZWdnZXIKaHR0cDovL3d3dy5yZW5lbnlmZmVuZWdnZXIuY2gKcGFzc2lvbiBmb3IgZGF0YQo=") {
    //     std::cout << "Encoding is wrong" << std::endl;
    //     all_tests_passed = false;
    // }

    // if (decoded != orig) {
    //     std::cout << "decoded != input line" << std::endl;
    //     all_tests_passed = false;
    // }

    // if (all_tests_passed) {
    //     std::cout << "test passed" << std::endl;
    // } else {
    //     std::cout << "test failed" << std::endl;
    // }

    return 0;
}