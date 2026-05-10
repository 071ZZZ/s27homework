import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

def kalman_filter(data, process_noise, measurement_noise):
    n = len(data)
    x = data[0]
    P = 100.0
    q = process_noise**2
    r = measurement_noise**2

    filtered = [x]
    for i in range(1, n):
        x_pred = x
        P_pred = P + q
        z = data[i]
        K = P_pred / (P_pred + r)
        x = x_pred + K * (z - x_pred)
        P = (1 - K) * P_pred
        filtered.append(x)
    return np.array(filtered)
if __name__ == "__main__":
    files = [
        "homework_1homework_data_1.csv",
        "homework_1homework_data_2.csv",
        "homework_1homework_data_3.csv",
        "homework_1homework_data_4.csv"
    ]
    titles = ["Data 1", "Data 2", "Data 3", "Data 4"]

    for i, fname in enumerate(files):
        try:
            data = np.loadtxt(fname, delimiter=',', skiprows=1)
        except FileNotFoundError:
            print(f"Warning: {fname} not found, skipping.")
            continue

        t = data[:, 0]          # 时间列
        y_obs = data[:, 1]      # 原始观测值
        fig, ax = plt.subplots(figsize=(12, 6))
        plt.subplots_adjust(bottom=0.22)

        line_orig, = ax.plot(t, y_obs, 'b.', markersize=1.5, alpha=0.5, label='Original')

        init_q = 0.5
        init_r = 1.0
        y_filt = kalman_filter(y_obs, init_q, init_r)
        line_filt, = ax.plot(t, y_filt, 'r-', linewidth=1.5, label='Filtered')

        ax.set_title(f'{titles[i]}  -  Process Noise: {init_q:.2f}  |  Measurement Noise: {init_r:.2f}')
        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.legend()
        ax.grid(True)

        ax_q = plt.axes([0.15, 0.10, 0.65, 0.03])
        ax_r = plt.axes([0.15, 0.05, 0.65, 0.03])

        slider_q = Slider(ax_q, 'Process Noise', 0.01, 5.0, valinit=init_q, valstep=0.01)
        slider_r = Slider(ax_r, 'Measurement Noise', 0.1, 10.0, valinit=init_r, valstep=0.1)

        def update(val):
            q_val = slider_q.val
            r_val = slider_r.val
            new_filt = kalman_filter(y_obs, q_val, r_val)
            line_filt.set_ydata(new_filt)
            ax.set_title(f'{titles[i]}  -  Process Noise: {q_val:.2f}  |  Measurement Noise: {r_val:.2f}')
            fig.canvas.draw_idle()

        slider_q.on_changed(update)
        slider_r.on_changed(update)

        # 按 's' 键保存当前画面
        def on_key(event):
            if event.key == 's':
                out_name = f"plot_tuned_{i+1}.png"
                fig.savefig(out_name, dpi=150, bbox_inches='tight')
                print(f"Saved {out_name} with current parameters")

        fig.canvas.mpl_connect('key_press_event', on_key)

        plt.show()