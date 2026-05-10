import pandas as pd
import matplotlib.pyplot as plt

# 读取结果
data = pd.read_csv("result.csv")

plt.figure(figsize=(12,6))
plt.scatter(
    data[data["Day"] <= 100]["Day"],
    data[data["Day"] <= 100]["Original"],
    s=12,
    label="Original Price"
)
plt.plot(
    data[data["Day"] <= 100]["Day"],
    data[data["Day"] <= 100]["Filtered"],
    linewidth=2,
    label="Kalman Filter"
)
plt.plot(
    data[data["Day"] > 100]["Day"],
    data[data["Day"] > 100]["Prediction"],
    linestyle="--",
    linewidth=2,
    color="red",
    label="Future Prediction (101~110)"
)
plt.xlabel("Day")
plt.ylabel("Price")
plt.title("Stock Price Prediction using Kalman Filter")
plt.legend()
plt.grid(True)
plt.show()
