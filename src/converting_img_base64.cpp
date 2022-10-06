#include <fstream>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "converting_base64/base64.h"

int main(int argc, char** argv) {
    bool all_tests_passed = true;
    std::string line;
    char c;
    std::string str_c;

    std::string filepath = ros::package::getPath("encode_n_decode_test");

    std::ifstream input(filepath + "/maps/Circle.png", std::ios::in | std::ios::binary);
    std::ofstream output(filepath + "/output/output.txt");
    std::ofstream output_pgm(filepath + "/maps/decoded_map.pgm");

    if(input.is_open()) {
        /// open file and saved as one variable for encoding once
        while(input.get(c)) {
            str_c.push_back(c);
        }
        std::string encoded_str = base64_encode(reinterpret_cast<const unsigned char*>(str_c.c_str()), str_c.length());
        std::string decoded_str = base64_decode(encoded_str);

        // compare with encoded string from Circle.png image. test_str is hard coded.
        std::string test_str = "iVBORw0KGgoAAAANSUhEUgAAABAAAAAQCAYAAAAf8/9hAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsMAAA7DAcdvqGQAAAFNSURBVDhPnVMxqoNAEH2KRQoLy7RCihQWQoQ0KWKfLpYewTt4AA8h9t7B1sKAoKWQQg8gpExg/4yu+f/z9weSB0/HcWd2Z+atJgj4gWEYkOc5LpcL6rqefK7rYrfb4XQ6YbPZTL4nOMGCNE2FZVmcUMnVaiWSJBH3+11G0ObyLcIwVAapeDwen0mmBLyzauErxnE8J+j7/t9jF0Uhrter8p9hGKJpGqFzw8ZxJN97eDweyLIMOnf7U/CUNMdxRNu20gXQsaU1j486j7IspQfwfV9aAJUOjeYquq6TLoBqlhawXq9BtU7aWGDbtrQA0zSBIAiUTWK+aiLzcDgIfb/fk/0ZPM+DzvLkOj/B+XymcxBYnvT9FqMomoXED5Yly1O1UMXtditut9t3AgYnYXmywlRBC3nnJZjx5zqzOFhhVVX9us7cMK6ZOj/5ZgBf9O4tfq8UiUsAAAAASUVORK5CYII=";
        ROS_INFO("output_length: %ld", encoded_str.size());
        ROS_INFO("teststr_length: %ld", test_str.size());

        if (decoded_str != str_c) {
            std::cout << "decoded_str is not the same as input line" << std::endl;
            all_tests_passed = false;
        } else if (encoded_str.size() != test_str.size()) {
            all_tests_passed = false;
        }

        /// write file
        output << encoded_str;
        output_pgm << decoded_str; // ad

        /// in order to read file as line by line, uncommented below
        /// It might lead to the result which is not compatible with other system or lanuage such as windows, c#
        /// But it works fine when decoding is done by this package
        // while(getline(input, line)) {
        //     std::string encoded_str = base64_encode(reinterpret_cast<const unsigned char*>(line.c_str()), line.length());
        //     // std::string decoded_str = base64_decode(encoded_str, true);

        //     if (decoded_str != line) {
        //         std::cout << "decoded_str is not the same as input line" << std::endl;
        //         all_tests_passed = false;
        //     }

        //     /// 필요없는 기능 ,,, 하지만 char 와 const char 포인터와 비교할 경우 알아두기 참고
        //     // int last_of_array = encoded_str.size();
        //     // ROS_INFO("encoded_size: %d", encoded_str.size());
        //     // ROS_INFO("%d", encoded_str[last_of_array - 1]);
        //     // char lastword = encoded_str[last_of_array -1];
        //     // if (lastword == *("=")) {
        //     //     ROS_INFO("= catched");
        //     //     break_point_cnt++;
        //     // }

        //     // write file
        //     output << encoded_str;
        //     // client에서 =기호가 호환이 안 되는 것 같아서 줄 바꿈시에 = 기호 제거 // base64_encode() 함수 내용 수정 -- converting_base64 pkg
        //     output_pgm << decoded_str + "\n"; // add linebreak  // 복원했을 때 이미지가 다시 제대로 표현 안되는 문제, line 들이 엔터가 안되는 현상
        // }
        input.close();
    } else {
        ROS_ERROR("Opening file is failed");
        return 1;
    }

    if (all_tests_passed) {
        ROS_WARN("test PASSED");
    } else {
        ROS_ERROR("test FAILED");
    }

    ////// original test code /////////
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
