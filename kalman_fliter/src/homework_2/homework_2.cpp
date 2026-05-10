#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

int main()
{
    vector<double> prices;
    ifstream file("stock_prices.csv");

    if (!file.is_open())
    {
        cout << "Failed to open stock_prices.csv" << endl;
        return -1;
    }

    string line;

    getline(file, line);

    while (getline(file, line))
    {
        stringstream ss(line);

        string day_str;
        string price_str;

        getline(ss, day_str, ',');
        getline(ss, price_str, ',');

        prices.push_back(stod(price_str));
    }

    file.close();
    double x[2];

    x[0] = prices[0];
    x[1] = 0;
    // 协方差矩阵 P
    double P[2][2] = {
        {1, 0},
        {0, 1}
    };
    // 状态转移矩阵 A
    double A[2][2] = {
        {1, 1},
        {0, 1}
    };
    // 观测矩阵 H
    double H[2] = {1, 0};
    // 过程噪声 Q
    double Q[2][2] = {
        {0.01, 0},
        {0, 0.01}
    };
    // 观测噪声 R
    double R = 0.1;
    vector<double> filtered_prices;
    vector<double> future_predictions;
    // 卡尔曼滤波
    for (double z : prices)
    {
        // 预测步骤
        double x_pred[2];

        x_pred[0] =
            A[0][0] * x[0] +
            A[0][1] * x[1];

        x_pred[1] =
            A[1][0] * x[0] +
            A[1][1] * x[1];
        // 协方差预测
        double P_pred[2][2];

        P_pred[0][0] =
            P[0][0] + P[0][1] +
            P[1][0] + P[1][1] +
            Q[0][0];

        P_pred[0][1] =
            P[0][1] + P[1][1];

        P_pred[1][0] =
            P[1][0] + P[1][1];

        P_pred[1][1] =
            P[1][1] + Q[1][1];

        // 卡尔曼增益
        double S = P_pred[0][0] + R;

        double K[2];

        K[0] = P_pred[0][0] / S;
        K[1] = P_pred[1][0] / S;

        double y = z - x_pred[0];

        x[0] = x_pred[0] + K[0] * y;
        x[1] = x_pred[1] + K[1] * y;

        P[0][0] =
            (1 - K[0]) * P_pred[0][0];

        P[0][1] =
            (1 - K[0]) * P_pred[0][1];

        P[1][0] =
            P_pred[1][0] -
            K[1] * P_pred[0][0];

        P[1][1] =
            P_pred[1][1] -
            K[1] * P_pred[0][1];

        filtered_prices.push_back(x[0]);
    }
    // 未来10天预测
    double future_x[2];

    future_x[0] = x[0];
    future_x[1] = x[1];

    for (int i = 0; i < 10; i++)
    {
        double next_price =
            A[0][0] * future_x[0] +
            A[0][1] * future_x[1];

        double next_velocity =
            A[1][0] * future_x[0] +
            A[1][1] * future_x[1];

        future_x[0] = next_price;
        future_x[1] = next_velocity;

        future_predictions.push_back(next_price);
    }
    ofstream out("result.csv");

    out << "Day,Original,Filtered,Prediction\n";

    for (int i = 0; i < prices.size(); i++)
    {
        out << i + 1 << ","
            << prices[i] << ","
            << filtered_prices[i] << ","
            << 0
            << "\n";
    }

    for (int i = 0; i < future_predictions.size(); i++)
    {
        out << prices.size() + i + 1 << ","
            << 0 << ","
            << 0 << ","
            << future_predictions[i]
            << "\n";
    }

    out.close();

    cout << "Kalman filter completed!" << endl;
    cout << "Result saved to result.csv" << endl;

    return 0;
}