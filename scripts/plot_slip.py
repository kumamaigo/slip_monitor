#!/usr/bin/env python3
import argparse
import glob
import os
import sys
import matplotlib.pyplot as plt
import pandas as pd

def plot_slip_log(csv_file):
  if not os.path.exists(csv_file):
    print(f"エラー: CSVファイル '{csv_file}' が見つかりません。")
    return

  df = pd.read_csv(csv_file)
  if df.empty:
    return

  t = df['time'] - df['time'].iloc[0]

  fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

  axes[0].plot(t, df['v_l_wheel'], label='v_l_wheel', color='blue')
  axes[0].plot(t, df['v_l_real'], label='v_l_real', color='deepskyblue', linestyle='--')
  axes[0].plot(t, df['v_r_wheel'], label='v_r_wheel', color='red')
  axes[0].plot(t, df['v_r_real'], label='v_r_real', color='orange', linestyle='--')
  axes[0].set_ylabel('Velocity [m/s]')
  axes[0].set_title('Drive Wheel Speed vs Ground Real Speed')
  axes[0].grid(True)
  axes[0].legend(loc='upper right')

  axes[1].plot(t, df['slip_l'], label='slip_l', color='blue')
  axes[1].plot(t, df['slip_r'], label='slip_r', color='red')
  axes[1].axhline(0.2, color='gray', linestyle=':')
  axes[1].axhline(-0.2, color='gray', linestyle=':')
  axes[1].set_ylabel('Slip Ratio')
  axes[1].set_ylim(-1.0, 1.0)
  axes[1].set_title('Wheel Slip Ratio')
  axes[1].grid(True)
  axes[1].legend(loc='upper right')

  axes[2].plot(t, df['omega_wheel'], label='omega_wheel', color='green')
  axes[2].plot(t, df['omega_imu'], label='omega_imu', color='purple', linestyle='--')
  axes[2].plot(t, df['spin_diff'], label='spin_diff', color='magenta')
  axes[2].set_xlabel('Time [s]')
  axes[2].set_ylabel('Angular Vel [rad/s]')
  axes[2].set_title('Yaw Rate & Spin Indicator')
  axes[2].grid(True)
  axes[2].legend(loc='upper right')

  plt.tight_layout()
  output_png = os.path.splitext(csv_file)[0] + '_plot.png'
  plt.savefig(output_png, dpi=300)
  print(f"【グラフ保存完了】: {output_png}")
  plt.show()

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('csv_file', nargs='?', default=None)
  args = parser.parse_args()

  target_csv = args.csv_file
  if target_csv is None:
    csv_files = sorted(glob.glob('slip_log_*.csv'))
    if not csv_files:
      print("CSVファイルが見つかりません。")
      sys.exit(1)
    target_csv = csv_files[-1]

  plot_slip_log(target_csv)
