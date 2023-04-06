### 起動するとき

1. トップディレクトリで下記のコマンドを入力  
   `docker build -t drone_python .`
2. イメージのビルド（1 のコマンド）が完了したら下記のコマンドでコンテナをスタートする。  
   `docker-compose up -d`
3. 続いて vscode の extension である Dev-Container 経由で work ディレクトリを開く

4. また、compiler を選ぶ際は Python3.11.2, Global Env とついているものを選ぶ
