// 参考：https://qiita.com/Qiita/items/c686397e4a0f4f11683d


## Dockerコマンド集


### 指定したディレクトリにあるdockerfileを探して実行する
**ビルド時**

`docker build -t <タグ名> <ディレクトリ>`

サンプルコード
```shell:sample1.sh
docker build -t <タグ名> <ディレクトリ>
```

というわけなので、`DockerFile`を作っておくと自分の好きなようにカスタマイズしDocker Imageをコマンド1行で作成できるようになる。ちなみに、このコマンド、基本的に時間かかるけどcacheを持ってくれることがあり、そうなると早い。

// このコマンドでコンテナ起動
docker container run -it python-practice 


---
`docker-compose`について

`docker-compose`を使うと複数のコンテナで構成されたアプリケーションについて、Docker Imageのビルドや各コンテナの起動・停止、をコマンド1行で実行できるようになる。

これがとても参考になる。
https://qiita.com/sugurutakahashi12345/items/0b1ceb92c9240aacca02