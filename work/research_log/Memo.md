## **研究のログ** 

---
#### 最初の仮説

* 空飛ぶ車では周囲の気流によって速度の調整が困難であるため、車よりも車間距離の調整が難しい。
* 
  * How to prove: 速度更新モデル（OV, IDV）などをベースにモデルを構築しても、それ＋αで車間距離が必要になることが予想される。従って、まず始めの研究として、車が2台あり、1台目が定常速度で運行、2台目がそれを追従する。というものを考える。なお、このとき、1台目・2台目ともに気流の影響を受け、速度の更新が車のようにいかない場合を想定する。

---

### シミュレーション条件

* ビークルは2台
* 1台目は気流の影響はうけれど定常運行する
* 2台目は最適速度モデルに従って1台目を追従する。
  * ここで何種類かを試す。
* 車列は一旦一列を想定する（究極的にはフルキャパ時はどこの車線も1列になるため）
* 周期境界/開放境界をどちらもやってみる。

---
参考: https://qiita.com/kamorits/items/6f342da395ad57468ae3