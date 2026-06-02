# HiWiFi — WiFi Mapper Spec

WiFiのRSSIとIMUを組み合わせて、屋内での自己位置推定とAP配置マップを描画するシステム。
目標: **移動に対して高レスポンス・高精度**。

---

## ハードウェア構成

| デバイス | 役割 |
|---|---|
| Arduino UNO R4 WiFi (RA4M1 + ESP32-S3) | IMU読み取り・WiFiスキャン・シリアル送信 |
| MPU-6050 (I2C @ 0x68, SCL=A4, SDA=A5) | 加速度(±2g) + ジャイロ(±250°/s) |
| HY-SRF05 | 超音波距離センサ（未使用、将来の壁検出用） |
| MacBook Pro + Processing | マップ描画・デッドレコニング |

---

## ファイル構成

```
HiWiFi/
├── Arduino/wifi_mapper/wifi_mapper.ino   # Arduino スケッチ
├── Processing/wifi_mapper/wifi_mapper.pde # Processing スケッチ
├── Devices.md   # ハードウェアリスト
├── Plan.md      # 改善機会リスト（ボトルネック分析）
└── Spec.md      # このファイル
```

---

## シリアルプロトコル (115200 baud, JSON改行区切り)

Arduino → Processing の一方向通信。

### `"i"` — IMUデータ (50ms毎)
```json
{"t":"i","t_ms":12345,"gz":0.012,"ax":-0.23,"ay":0.45,"mv":1}
```
| フィールド | 単位 | 説明 |
|---|---|---|
| `gz` | rad/s | ヨー角速度（キャリブ済み） |
| `ax`, `ay` | m/s² | 水平加速度（重力補正・キャリブ済み） |
| `mv` | 0/1 | 移動判定 (水平加速度mag > 0.35 m/s²) |

### `"scan"` — スキャン状態
```json
{"t":"scan","state":"begin","t_ms":12345}
{"t":"scan","state":"end","t_ms":14200,"n":8}
```

### `"s"` — WiFiスキャン結果
```json
{"t":"s","t_ms":14100,"a":[{"n":"SSID","b":"AA:BB:CC:DD:EE:FF","r":-65,"c":6},...]}
```

### `"fb"` — FIFOバースト（スキャン中のIMUデータ）★ 新規追加
```json
{"t":"fb","ts":12000,"te":14100,"dt":40,"n":42,"ov":0,"d":[ax0,ay0,gz0,ax1,ay1,gz1,...]}
```
| フィールド | 説明 |
|---|---|
| `ts` | FIFO_RESET発行時刻 (スキャン開始直前) |
| `te` | stopFifo完了時刻 |
| `dt` | サンプル間隔ms (固定40 = 25Hz) |
| `n` | サンプル数 (最大64 = 2.56秒分) |
| `ov` | 1=FIFOオーバーフロー発生 (スキャンが2.56秒超) |
| `d` | [ax,ay,gz]×n の平坦配列、キャリブ済み (m/s², rad/s) |

---

## 実装済み機能

### Arduino

- MPU-6050 初期化・キャリブレーション (200サンプル平均)
- IMU送信 20Hz (IMU_INTERVAL_MS=50)
- WiFiスキャン 2秒毎 (SCAN_INTERVAL_MS=2000)、最大20AP
- **MPU-6050 FIFO バッファリング** ★ 実装済み
  - スキャン前: `startFifo()` で25Hz ODR設定・FIFO有効化
  - スキャン中: WiFiS3ブロック中もFIFO自律蓄積 (最大2.56秒=64サンプル)
  - スキャン後: `stopFifo()` → `drainAndSendFifo()` で "fb" パケット送信
  - I2C 400kHz (`Wire.setClock(400000)`) でドレイン高速化

### Processing

- シリアル受信 → JSONパース → apMap更新
- MotionState: heading積分 + 速度モデル + 加速度LPF
- **スキャン中FIFOデッドタイム補正** ★ 実装済み
  - "begin" 受信時: `saveSnapshot()` でMotionState全フィールド保存
  - "s" パケット: バッファリング（APは補正後に正確な位置で配置）
  - "fb" 受信: `pendingFifoBurst` にセット（draw()スレッドで処理）
  - `applyFifoBurst()`: スナップショット巻き戻し→全FIFOサンプル再生→補正位置確定
  - 補正lerp: alpha=0.25/frame、約10フレーム(167ms)で94%収束
  - デバッグ表示: `FIFO: corr/ok`
- キー操作: WASD=マップ移動、`<`/`>`=ズーム、Space=次スキャンを2点観測に記録、R=姿勢リセット、C=AP/2点観測クリア、G=デバッグ表示切替

---

## 主要定数一覧

### Arduino
| 定数 | 値 | 説明 |
|---|---|---|
| `IMU_INTERVAL_MS` | 50 | IMU送信間隔ms |
| `SCAN_INTERVAL_MS` | 2000 | WiFiスキャン間隔ms |
| `MAX_APS_PER_SCAN` | 20 | 1スキャンで送るAP上限 |
| `MOVE_THRESHOLD` | 0.35 | 移動判定閾値 m/s² |
| `CALIB_SAMPLES` | 200 | キャリブレーションサンプル数 |
| `FIFO_ODR_MS` | 40 | FIFOサンプル間隔ms (25Hz) |
| `FIFO_MAX_SAMPLES` | 64 | FIFO最大サンプル数 |

### Processing
| 定数 | 値 | 説明 |
|---|---|---|
| `METERS_TO_PX` | 40.0 | 初期ズーム: 1m = 40px |
| `MIN_METERS_TO_PX` / `MAX_METERS_TO_PX` | 12.0 / 160.0 | ズーム下限/上限 |
| `WALK_SPEED` | 0.8 | 移動時基準速度 m/s |
| `ACCEL_DEADBAND` | 0.12 | 加速度デッドバンド m/s² |
| `ACCEL_LPF_ALPHA` | 0.25 | 加速度LPF係数 |
| `ACCEL_GAIN` | 0.35 | 加速度→速度変換ゲイン |
| `MOVING_BLEND` | 0.09 | WALK_SPEEDへのブレンド係数 |
| `MOVING_DAMPING_PER_S` | 0.94 | 移動時速度減衰 |
| `STILL_DAMPING_PER_S` | 0.10 | 静止時速度減衰 |
| `IMU_STALE_MS` | 700 | IMU失効判定ms |
| `SCAN_PREDICT_MS` | 3000 | スキャン中予測継続ms |
| `ACCEL_HOLD_MS` | 350 | 加速度保持ms |
| `AP_DISTANCE_BLEND` | 0.20 | AP位置lerp係数 |
| `MOVE_THRESHOLD_MS2` | 0.35 | FIFO再生時の移動判定閾値 |
| `FIFO_CORRECT_ALPHA` | 0.25 | FIFO補正lerp係数 |

---

## 残課題・改善候補 (Plan.md より)

### 🔴 高優先

**B. IMUサンプリング高速化 ★ 実装済み**
- `IMU_INTERVAL_MS = 10` (20Hz→100Hz)
- Processing: gz × arduinoDt を serialEvent スレッドで累積 → draw() で消費（真の100Hz heading積分）
- シリアル帯域: 100Hz × ~64bytes/packet ≈ 6.4KB/s → 115200baud で余裕あり (55%)

**C. 静止時ジャイロバイアス補正 ★ 実装済み**
- `moving==0` 時に `gzBias = gzBias * 0.995f + gz * 0.005f` でEMA更新
- τ ≈ 2s@100Hz。長時間使用でのヨードリフト抑制

**A. PDR歩行検出 (未実装)**
- az (垂直加速度) のピーク検出で1歩カウント
- 1歩 = ストライド長(~0.65m) × heading方向
- 連続速度積分より大幅に安定

**D. WiFi三角測位でIMUドリフト補正 ★ 実装済み**
- `hitCount >= 3` の信頼AP 3つ以上で線形最小二乗三角測位
- 外れ値棄却（>5m）後、`TRILATERATION_BLEND=0.15` でlerp補正
- デバッグ表示: `TRIL anchors: N/M`

### 🟡 中優先

**E. HY-SRF05 超音波センサ活用 (未実装)**
- 壁検出 → 「壁から×m」制約でIMU誤差補正

**F. RSSIフィルタリング (未実装)**
- 同一APを複数回サンプリングして中央値/EMAで送信
- 生RSSIのノイズ ±5dBm を低減

**H. チャンネル別パスロスモデル (未実装)**
- 2.4GHz: n=2.5, A=-59dBm / 5GHz: n=2.0, A=-56dBm
- 現在は全チャンネル同一モデル

**I. カルマンフィルタ融合 (未実装)**
- 状態: [x, y, vx, vy, heading]
- 観測1: IMU (高頻度・ドリフトあり)
- 観測2: WiFi位置推定 (低頻度・大ノイズ)
- EKF or UKF で最適融合

### 🟢 低優先

**J. マップ保存/再ロード** — AP位置・経路をJSONで永続化  
**K. フィンガープリント法** — k-NN でWiFiのみ位置推定  
**L. UNO R4 LEDマトリクス** — スキャン/移動/静止をLED表示

---

## 既知の制約・注意事項

1. **WiFiS3は非同期スキャン不可** — `WiFi.scanNetworks()` は同期ブロック専用。FIFOで対応済み。
2. **FIFOバイト順** — AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, GZ_H, GZ_L (8bytes/sample、ハード固定)
3. **FIFO容量** — 512bytes / 8 = 64サンプル = 2.56秒。これを超えるスキャン（まれ）はovフラグで通知、末尾を予測補完
4. **Processing スレッド** — `serialEvent()` は別スレッド。`motion` 操作は `draw()` スレッドのみ (`pendingFifoBurst` 経由)。`apMap` は `synchronized(apMap)` で保護済み
5. **SERIAL_PORT要変更** — Processing の `SERIAL_PORT` 定数を実機に合わせる (`printArray(Serial.list())` で確認)
6. **キャリブレーション** — 起動時に静止状態が必要 (200サンプル × 5ms = 約1秒)

---

## ゴール

WiFiのAP群を地図上に配置し、ユーザが部屋を歩き回ることで実際の電波環境マップを構築する。将来的には複数のAP位置が収束し、RSSIだけで「今どこにいるか」が推定できる状態を目指す。
