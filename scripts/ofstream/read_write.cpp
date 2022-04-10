#include <iostream>
#include <sstream>  
#include<iomanip>

#include <memory>
#include <vector>

#include<boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main() {
    double timestamp = 123456.1244;
    std::string f_path = "/home/liuwch/OneDrive/dev/workspace/ofstream/data";
    std::stringstream ss;
    ss << std::setiosflags(std::ios::fixed)     // 计算浮点数且希望能控制其输出、精度、小数点后的位数等
            << std::setprecision(3) << timestamp << ".txt";
    std::cout << ss.str() << std::endl;
    fs::path image_path = fs::path(f_path);
    std::cout << image_path.string() << std::endl;
}
