#include <WiFi.h>

const char* ssid = "K187714";  // Wi-FiのSSID
const char* password = "0757+tF2";  // Wi-Fiのパスワード
const char* serverUrl = "10.0.0.208";  // Node.jsサーバーのIPアドレス
const int serverPort = 3000;  // Node.jsサーバーのポート

void setup() {
  Serial.begin(115200);  // シリアル通信を開始

  // Wi-Fi接続の開始
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // 1秒待つ
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  sendData();  // サーバーへのデータ送信
}

void loop() {}

void sendData() {
  WiFiClient client;  // WiFiClientインスタンスを作成

  // サーバーに接続
  if (client.connect(serverUrl, serverPort)) {
    Serial.println("Connected to server");

    // HTTPリクエストを送信
    client.println("POST /endpoint HTTP/1.1");
    client.println("Host: 10.0.0.208");  // サーバーのホスト名（またはIPアドレス）
    client.println("Content-Type: application/json");

    // JSONデータの長さをヘッダーに追加
    String jsonData = "{\"sensor_data\": 123, \"status\": \"active\"}";
    client.print("Content-Length: ");
    client.println(jsonData.length());  // Content-Lengthの設定
    client.println();  // ヘッダーとボディの間の空行
    client.println(jsonData);  // JSONデータを送信

    // サーバーからの応答をシリアルモニタに表示
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
    }
  } else {
    Serial.println("Connection to server failed");
  }

  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // ESP32のローカルIPアドレスを表示

  client.stop();  // サーバーとの接続を終了
}
