# slip_monitor

ROS 2 環境における**車輪スリップ率のリアルタイム監視・CSVログ保存**、および**車両限界加速度に基づく自動加減速テスト**を行うためのROS 2パッケージです。

---

## 概要

本パッケージは以下の3つのコンポーネントで構成されています。

1. **`slip_monitor_node` (C++)**: 
   CANバスからの駆動輪回転数と対地オドメトリ/IMUデータを統合し、左右輪のスリップ率およびスピン指標（$\Delta \omega$）を計算して CSV ログへリアルタイム保存します。
2. **`detection_node` (C++)**: 
   車両諸元と路面摩擦限界 $\mu$ から限界加速度・減速度を計算し、ステートマシン駆動による自動加減速走行テストを実行します。
3. **`plot_slip.py` (Python)**: 
   計測後に生成された CSV ログを解析し、「車輪速/対地速度」「スリップ率」「スピン指標」の 3 段グラフを画面表示および PNG 画像として自動出力します。

---

## ディレクトリ構成

```text
slip_monitor/
├── CMakeLists.txt              # ビルド設定
├── package.xml                 # 依存関係定義
├── .gitignore                  # Git除外設定
├── README.md                   # 本ドキュメント
├── src/                        # C++ソースコード
│   ├── slip_monitor_node.cpp   # スリップ監視・CSVログ保存ノード
│   └── detection_node.cpp      # 加減速試験実行ノード
├── scripts/                    # Pythonツール
│   └── plot_slip.py            # ログ解析・グラフ描画スクリプト
└── launch/                     # ランチファイル
    └── slip_monitor.launch.py  # 一括起動用スクリプト
