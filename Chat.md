```
#include <WiFi.h>で何ができる？ 幅広く知りたい
```

`#include <WiFi.h>` は ESP32 の Wi-Fi機能全般を扱うためのライブラリです。

大きく分けると以下ができます。

# 1. Wi-Fi接続

自宅や大学のWi-Fiへ接続

```cpp
WiFi.begin(ssid, password);
```

取得可能:

* IPアドレス
* ゲートウェイ
* DNS
* 接続状態

```cpp
WiFi.localIP();
WiFi.gatewayIP();
WiFi.status();
```

---

# 2. Wi-Fiスキャン

周囲のアクセスポイントを検出

```cpp
WiFi.scanNetworks();
```

取得可能:

* SSID
* RSSI（電波強度）
* BSSID(MAC)
* チャンネル
* 暗号化方式

例:

```
HomeWiFi   -55 dBm
iPhone     -72 dBm
eduroam    -60 dBm
```

---

# 3. アクセスポイント化

ESP32自身をWi-Fiルーターにできる

```cpp
WiFi.softAP("MyESP32");
```

スマホから接続可能

用途:

* ロボット設定画面
* IoT初期設定
* ファイル転送

---

# 4. Webサーバ

ESP32をサーバ化

```cpp
WiFiServer server(80);
```

ブラウザから

```
http://192.168.x.x
```

でアクセス

用途:

* センサ監視
* ロボット操作
* カメラ映像

---

# 5. Webクライアント

ESP32からサイトへアクセス

```cpp
WiFiClient client;
```

例

```cpp
client.connect("example.com",80);
```

用途:

* API取得
* 天気情報
* AIサービス利用

---

# 6. TCP通信

ソケット通信

```cpp
WiFiClient
WiFiServer
```

可能

用途:

* PCとの通信
* ロボット間通信
* ゲームサーバ

---

# 7. UDP通信

高速通信

```cpp
WiFiUDP udp;
```

用途:

* ドローン
* センサ配信
* リアルタイム位置共有

---

# 8. RSSI取得

現在の電波強度

```cpp
WiFi.RSSI();
```

例

```
-30 強い
-60 普通
-90 弱い
```

用途:

* 屋内測位
* 接近検知
* 電波地図

---

# 9. MACアドレス取得

```cpp
WiFi.macAddress();
```

取得例

```
A0:B7:65:12:34:56
```

---

# 10. IPアドレス管理

```cpp
WiFi.localIP();
WiFi.subnetMask();
WiFi.gatewayIP();
```

ネットワーク解析に利用可能

---

# 11. NTP時刻同期

Wi-Fi経由で正確な時刻取得

```cpp
configTime(...)
```

取得例

```
2026-06-01 22:30:00
```

用途:

* ログ
* データ収集

---

# 12. mDNS

IPを覚えなくても接続

```cpp
esp32.local
```

でアクセス可能

用途:

* ローカル開発
* 家庭内IoT

---

# 13. Wi-Fiイベント取得

接続・切断検知

```cpp
WiFi.onEvent(...)
```

検出例

* 接続
* 切断
* IP取得
* AP接続

---

# 14. AP接続者取得

アクセスポイントモード時

接続中端末数を把握

```cpp
WiFi.softAPgetStationNum();
```

---

# 15. ESP32同士通信

ESP32 A ⇔ ESP32 B

* TCP
* UDP
* HTTP

で直接通信可能

---

# 16. 位置推定・電波測位

周囲のWi-Fiを観測

```cpp
SSID
BSSID
RSSI
```

から

* 室内位置推定
* 地図作成
* 自己位置推定

が可能

---

# 17. Wi-Fiパケット監視（高度）

ESP32にはプロミスキャスモードがあります。

```cpp
esp_wifi_set_promiscuous(true);
```

受信可能:

* Beacon
* Probe Request
* Probe Response
* Management Frame

用途:

* 人流計測
* デバイス検出
* 無線研究

---

# 18. AI・クラウド連携

Wi-Fi経由で

* [OpenAI](https://openai.com?utm_source=chatgpt.com)
* [Google Cloud](https://cloud.google.com?utm_source=chatgpt.com)
* [AWS IoT Core](https://aws.amazon.com/iot-core/?utm_source=chatgpt.com)

などへ接続可能

用途:

* 音声認識
* 画像認識
* LLMエージェント

---

ESP32では `WiFi.h` だけでも、

* 周囲の無線環境を観測する「センサ」
* Webサーバ
* ネットワーククライアント
* Wi-Fiルーター
* 屋内測位装置
* 人流センサ
* IoTゲートウェイ

として利用できます。特に「周囲の電波を観測して現実世界を認識する」用途は、ロボットや世界モデルとの相性が良い分野です。


```
Wi-Fiレーダー
Wi-Fiヒートマップ　などはいいアイデアだ

位置測定も絡められるのなら素晴らしい

使用できるのは以下

Arduino UNO R3
Arduino UNO R4 Wi-Fi
HY-SRF05（超音波センサ）
MPU-6050（ジャイロセンサ）
HW-504（ジョイスティックモジュール）
その他スイッチ、ジャンパ、基盤、LEDなど

Macbook pro
ElecomのWi-Fiルータ
iPhone SE 2

どのようなことができるか、自由にたくさんアイデアを出して欲しい
```


手持ちの機材だけでも、「Wi-Fiを使って空間を観測する」系のプロジェクトはかなりできます。

ただし重要なのは、

* Arduino UNO R3 → Wi-Fiなし
* Arduino UNO R4 WiFi → Wi-Fiあり（ESP32-S3搭載）

なので、Wi-Fi関連は基本的に UNO R4 WiFi を使うことになります。

---

# 1. Wi-Fiヒートマップ作成機

部屋を歩き回る。

各地点で

* RSSI
* 座標

を記録。

結果

```text
強い
█████████

中
█████

弱い
██
```

のような電波マップを作る。

発展

* 部屋の間取りを描画
* Blenderで3D可視化
* Unityでヒートマップ表示

---

# 2. Wi-Fi等高線マップ

ヒートマップをさらに発展。

地図上に

```text
-40dBm
-50dBm
-60dBm
-70dBm
```

の等高線を描く。

まるで地形図。

---

# 3. Wi-Fiコンパス

ルータ方向を推定。

方法

* 少し移動
* RSSI変化観測

```text
前進 → 強くなった
右移動 → 弱くなった
```

ならルータは前方。

---

# 4. Wi-Fi宝探し

ルータを隠す。

Arduinoで

```text
ピッ
ピッ
ピピピピ
```

とRSSIに応じて音を鳴らす。

金属探知機風。

---

# 5. Wi-Fiレーダー

周囲のSSIDを検出。

表示例

```text
Router
iPhone
MacBook
```

強度順に並べる。

---

# 6. Wi-Fi生態系マップ

一日中スキャン。

時間ごとの

* AP数
* RSSI

変化を記録。

例えば

```text
朝 5個
昼 15個
夜 30個
```

マンションの活動状況が見える。

---

# 7. 人流センサ

スマホのWi-Fiビーコン観測。

通過人数推定。

大学の廊下などで面白い。

---

# 8. Wi-Fi気象観測

RSSIの時間変化を記録。

人体や家具移動で変化。

```text
誰か通った
↓
RSSI変化
```

---

# 9. 部屋の電波3Dスキャン

高さも変えて測定。

```text
床
机
頭上
```

で比較。

意外と分布が違う。

---

# 10. Wi-Fi SLAM

かなり研究寄り。

Wi-Fiをランドマークとして使う。

取得

* BSSID
* RSSI

例

```text
AP1 -45
AP2 -60
AP3 -70
```

この組み合わせが場所の指紋になる。

---

# 11. GPS無し位置推定

室内版GPS。

特徴量

```text
AP1=-45
AP2=-62
AP3=-81
```

↓

場所A

学習。

---

# 12. Wi-Fi地図生成AI

部屋を歩き回る。

収集

```text
位置
RSSI
```

↓

機械学習

↓

未知位置推定。

MacBookで実装可能。

---

# 13. Wi-Fiナビゲーション

目的地での電波パターンを保存。

現在との差を比較。

```text
あと5m
```

みたいな案内。

---

# 14. Wi-Fiを使うロボット犬

ジョイスティックで操縦。

ルータを餌とみなす。

RSSIが強くなる方向へ進む。

---

# 15. Wi-Fi磁力線可視化

RSSIをベクトル場として描く。

かなりアート寄り。

---

# 16. 電波の影を見る

人が間に立つ。

```text
RSSI
↓
3dB低下
```

人体が電波を遮る様子を可視化。

---

# 17. 超音波＋Wi-Fi融合測位

HY-SRF05使用。

取得

* 壁まで距離
* RSSI

両方使う。

SLAMの簡易版。

---

# 18. IMU＋Wi-Fiデッドレコニング

MPU6050使用。

推定

```text
歩いた距離
向き
```

↓

Wi-Fiで補正。

実際のロボット研究で使われる発想。

---

# 19. 電波地形探索ゲーム

部屋をダンジョン化。

電波が強い場所

↓

回復ポイント

弱い場所

↓

危険地帯

---

# 20. Wi-Fi顕微鏡

ルータ周辺を数cm刻みで測定。

すると

```text
-45
-48
-43
-51
```

のように激しく変化する。

マルチパス（反射）の観察。

---

# 21. Wi-Fiで家具配置最適化

机を動かす。

本棚を置く。

その前後のRSSIを測定。

最も電波が届くレイアウトを探索。

---

# 22. Wi-Fi世界モデル

特に面白い。

空間を

```text
位置
↓
Wi-Fi特徴量
```

で表現。

例えば

```text
入口
AP1=-50
AP2=-70

机
AP1=-42
AP2=-60
```

となる。

これは「空間をWi-Fiで記述した地図」。

ユーザーが興味を持っている世界モデルにかなり近い。

---

個人的に最も発展性が高いのは

1. Wi-Fiヒートマップ
2. Wi-Fi指紋測位（Fingerprint Localization）
3. IMU＋Wi-Fi自己位置推定
4. Wi-Fi SLAM
5. Wi-Fi世界モデル

です。

特に「Wi-Fi指紋測位」は、手持ち機材だけで研究テーマレベルの内容に発展できます。Arduino UNO R4 WiFi と MacBook があれば、部屋のどこにいるかを Wi-Fi のみから推定するシステムを構築できます。
