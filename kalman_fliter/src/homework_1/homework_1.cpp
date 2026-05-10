#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>

struct DataPoint {
    double t;
    double y;
};

std::vector<DataPoint> readData(const std::string& filename) {
    std::vector<DataPoint> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << "\n";
        return data;
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        double t, y;
        if (iss >> t >> y) {
            data.push_back({t, y});
        }
    }
    file.close();
    return data;
}

std::vector<double> kalmanFilter(const std::vector<DataPoint>& data,
                                   double processNoise = 0.5, double measurementNoise = 1.0) {
    size_t n = data.size();
    if (n == 0) return {};

    double pos = data[0].y;     
    double p = 100.0;           

    double q = processNoise * processNoise;   // 过程噪声方差
    double r = measurementNoise * measurementNoise; // 测量噪声方差

    std::vector<double> smoothPos;
    smoothPos.reserve(n);
    smoothPos.push_back(pos);

    for (size_t i = 1; i < n; ++i) {

        double predPos = pos;
        double predP = p + q;

        double y = data[i].y;
        double S = predP + r;
        double K = predP / S;
        pos = predPos + K * (y - predPos);
        p = (1 - K) * predP;

        smoothPos.push_back(pos);
    }
    return smoothPos;
}

int main() {
    std::vector<std::string> files = {
        "homework_data_1.txt",
        "homework_data_2.txt",
        "homework_data_3.txt",
        "homework_data_4.txt"
    };
    std::vector<double> dts = {0.001, 0.01, 1.0, 0.001};   // 采样间隔

    for (size_t f = 0; f < files.size(); ++f) {
        std::string filename = files[f];
        double dt = dts[f];
        std::cout << "\nProcessing " << filename << " ...\n";

        std::vector<DataPoint> data = readData(filename);
        if (data.empty()) continue;
        std::cout << "Data points: " << data.size() << "\n";

        // 提取时间和观测值
        std::vector<double> t(data.size()), yObs(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            t[i] = data[i].t;
            yObs[i] = data[i].y;
        }

        // 卡尔曼滤波
        std::vector<double> ySmooth = kalmanFilter(data, 0.1, 2.3);
        std::cout << "Kalman filter done.\n";

        // 将结果保存为 CSV
        size_t dotPos = filename.find_last_of('.');
        std::string baseName = (dotPos == std::string::npos) ? filename : filename.substr(0, dotPos);
        std::string outFile = "homework_1" + baseName + ".csv";
        std::ofstream fout(outFile);
        fout << "t,original_y,smoothed_y\n";  // 移除了 fitted_y
        for (size_t i = 0; i < data.size(); ++i) {
            fout << t[i] << "," << yObs[i] << "," << ySmooth[i] << "\n";
        }
        fout.close();
        std::cout << "Results saved to " << outFile << "\n";
    }

    std::cout << "\nAll done. Now run plot.py to generate figures.\n";
    return 0;
}
