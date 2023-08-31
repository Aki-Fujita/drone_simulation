### 立ち上げ方

- 開始時
```docker-compose up```を実行

---
#### エラー対処方法
- `libGL.so.1: cannot open shared object file` => これはcv2を読み込む際に出ることがある。これが出た場合はコンテナ内で以下のコマンドを実行:
  ```sudo apt-get install libgl1-mesa-glx```