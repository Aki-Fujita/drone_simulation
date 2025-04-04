import copy
from typing import List, Dict, Any


"""
DFRのシミュレーションで利用する acc_itineraryのためのクラス. 

"""
class AccItinerary:
    def __init__(self, acc_itinerary):
        """
        acc_itinerary は各 accel オブジェクト（辞書）のリストで、
        各オブジェクトは以下のキーを持つ:
         - 't_start': float  （加速開始時刻）
         - 'acc': float      （加速度）
         - 'x_start': float  （開始位置）
         - 'v_0': float      （初速度）
         - 't_end': float    （終了時刻）
        """
        self._itinerary: List[Dict[str, float]] = copy.deepcopy(acc_itinerary)
        self.acc = 0
    
    def itinerary(self):
        """現在のacc_itineraryを返す"""
        return self._itinerary
    
    # 下の関数をアップデートしたもの. 
    def sudden_insert(self, new_accel):
        """
        _sudden_insertには課題があり、それは
        ・加速中だった場合に挿入されると、挿入後にスピードが予定されていたところに元に戻らない
        →そのためETAが過剰に遅くなってしまう、という課題があった. 
        """
        new_t_start = new_accel[0]["t_start"]
        new_t_end = new_accel[-1]["t_end"]
        new_itinerary: List[Dict[str, float]] = []

        for segment in self._itinerary:
            if segment["t_end"] <= new_t_start:
                new_itinerary.append(segment)
                continue
            # あるセグメントの途中から挿入される場合. 
            # この場合はsegmentを分割して、後半部は時間をずらして後で差し込む（速度誤差を消すため）. 
            elif segment["t_start"]< new_t_start and new_t_start < segment["t_end"]:
                former_segment = {**segment, "t_end": new_t_start}
                new_itinerary.append(former_segment)
                new_itinerary += new_accel
                last_segment = new_accel[-1]
                last_segment_delta = last_segment["t_end"] - last_segment["t_start"]

                x_start_latter = last_segment["x_start"] + last_segment["v_0"] * last_segment_delta + 0.5 * last_segment["acc"] * last_segment_delta**2
                t_end_latter = new_t_end + (segment["t_end"] - new_t_start)
                latter_segment = {**segment, "t_start": new_t_end, "x_start": x_start_latter, "t_end":t_end_latter}

                new_itinerary.append(latter_segment)
            
            elif segment["t_start"] >= new_t_start:
                new_accel_period = new_t_end - new_t_start
                segment_t_start = segment["t_start"] + new_accel_period
                segment_t_end = segment["t_end"] + new_accel_period

                last_segment = new_itinerary[-1]
                last_segment_delta = last_segment["t_end"] - last_segment["t_start"]
                x_start = last_segment["x_start"] + last_segment["v_0"] * last_segment_delta + 0.5 * last_segment["acc"] * last_segment_delta**2
                if abs(segment["t_start"] -141.730) <0.01:
                    print("===DEBUG===")
                    print(last_segment)
                    print(x_start)
                    print("============")

                segment_to_add = {**segment, "t_start": segment_t_start, "t_end":segment_t_end, "x_start": x_start} # acc, v0は変わらない.
                new_itinerary.append(segment_to_add)
                
                continue
            
            else:
                print(new_itinerary)
                print(segment)
                raise ValueError("ここには来ないはず")
            
                
        self._itinerary = new_itinerary

    # 急ブレーキをかけるシミュレーションのために用意した関数. 逆にそこ以外では使わないようにする. 
    def _sudden_insert(self, new_accel):
        """
        急ブレーキをかけた時の挙動（減速=>加速となるようなacc_itineraryを挿入する, ブレーキ単体だとシミュレーション結果に影響がありそうなため.）. 
        accel オブジェクト new_accel を既存の acc_itinerary に挿入します。
        【注意】 計算内で、new_accelは挿入された瞬間の車のx座標, 速度(v)を保持していることを想定しているので実装時もそうするように気を付ける. 
        挙動は以下の通り:
        (a) 挿入されるaccObjより前の場合 => 気にしない（そのまま追加）
        (b) segmentの途中でnew_accelが挿入される場合
        (c) 挿入されるaccObjよりも完全に後の場合 => 加速度情報はこのままで良いが, v_0とx_startを整理する必要がある. 
        →このパターンはさらに分割が必要
        b-1: new_accが複数のsegmentにまたがる場合.
        => ・このセグメントはnew_t_startまでで一回切る. # b-1-1
           ・次のsegmentとしてnew_accelを追加.  # b-1-2
           ・ここまでが済んだら次のsegmentを見る. # b-1-3
           ・次のsegmentに対して、
               if t_end < new_t_end: => continue # b-1-3-1
               else:t_startをnew_t_endにした上で追加. # b-1-3-2
        b-2: 一つのsegment内にnew_accが収まる場合.
            ・t_start < t < new_t_startをappend # b-2-1
            ・続いてnew_accをappend # b-2-2
            ・最後にnew_t_end < t < t_endをappend  # b-2-3
        """
        # スケジュールは t_start の昇順にソートされている前提とする
        new_t_start = new_accel[0]["t_start"]
        new_t_end = new_accel[-1]["t_end"]

        new_itinerary: List[Dict[str, float]] = [] # ここには絶対に正しいものが入っている前提で進む.
        inserted = False

        for idx, current_acc_segment in enumerate(self._itinerary):
            # 挿入されるaccObjより前の場合. 
            if current_acc_segment["t_end"] <= new_t_start:
                # new_accel 以前のセグメントはそのまま
                new_itinerary.append(current_acc_segment)
                continue
            
            # これは完全に無視して良いケース. segmentがnew_accの中に完全に収まっている場合. b-1-3
            elif new_t_start <= current_acc_segment["t_start"]  and current_acc_segment["t_end"] <= new_t_end:
                continue

            # 挿入されるaccObjよりも完全に後ろなものの場合. 
            elif new_t_end < current_acc_segment["t_start"] :
                prev_seg = new_itinerary[-1] # previous segmentの略. 
                delta_t = prev_seg["t_end"] - prev_seg["t_start"]
                v_0 = prev_seg["v_0"] + prev_seg["acc"] * delta_t
                x_start = prev_seg["x_start"] + prev_seg["v_0"] *delta_t + 0.5 * prev_seg["acc"] * delta_t **2
                if v_0 < 0:
                    print("prev_seg: ", prev_seg)
                    raise ValueError("セグメント終わりに速度が0になっています")

                segment_to_add = {**current_acc_segment, "v_0":v_0, "x_start":x_start}
                new_itinerary.append(segment_to_add)
                continue
            
            # segmentの途中でnew_accelが挿入される場合.
            elif current_acc_segment["t_start"] <= new_t_end and new_t_start < current_acc_segment["t_end"]:
                # b-1 複数セグメントにまたがる場合（このsegmentのt_endを超える）
                if new_t_end > current_acc_segment["t_end"]:
                    this_segment = {**current_acc_segment, "t_end": new_t_start}
                    new_itinerary.append(this_segment) # b-1-1
                    new_itinerary += new_accel# b-1-2
                    continue
                
                # b-1-3の終端部の処理
                elif new_t_start < current_acc_segment["t_start"] and new_t_end <= current_acc_segment["t_end"]:
                    # b-1-2の段階でnew_itineraryにnew_accelがそもそもappendされているものとして処理を行う. 
                    # まずはnew_accelの最後における速度とx座標を計算. 
                    new_accel_last = new_accel[-1]
                    delta_t = new_accel_last["t_end"] - new_accel_last["t_start"]
                    v_0 = new_accel_last["v_0"] + new_accel_last["acc"] * delta_t
                    if v_0 > 0:
                        x_start = new_accel_last["x_start"] + new_accel_last["v_0"] * delta_t + 0.5 * new_accel_last["acc"] * delta_t ** 2
                    else:
                        raise ValueError("ブレーキの結果速度が0になりました")
                    
                    # 次のsegmentの出口時点で速度が0になる場合はそのまま追加できない. 
                    v_exit = v_0 + current_acc_segment["acc"] * (current_acc_segment["t_end"] - new_t_end)

                    if v_exit > 0:
                        this_segment = {"t_start": new_t_end, "acc": current_acc_segment["acc"], "v_0": v_0, "x_start": x_start, "t_end": current_acc_segment["t_end"]}
                        new_itinerary.append(this_segment)
                        continue
                    # 速度が0になる場合は基本的にはないので、その場合はエラーを出す.
                    else:
                        raise ValueError("ブレーキの結果速度が0になりました")
                        
                    
                
                # b-2 一つのセグメント内に収まる場合. 
                elif current_acc_segment["t_start"] <= new_t_start and new_t_end <= current_acc_segment["t_end"]:
                    this_segment = {**current_acc_segment, "t_end": new_t_start}
                    new_itinerary.append(this_segment) # b-2-1
                    new_itinerary += new_accel# b-2-2
                    
                    # 続いてsegment_latterを計算する. この際、segment_latter開始時点のv_0とx_startを計算する必要がある. 
                    # current_acc_segmentに挿入された後の形を計算. このとき、ソースとしてはnew_accelしか使わないのでnew_accelは挿入される時の状況を反映していないといけない.  
                    new_accel_last = new_accel[-1]
                    delta_t = new_accel_last["t_end"] - new_accel_last["t_start"]
                    v_0 = new_accel_last["v_0"] + new_accel_last["acc"] * delta_t
                    if v_0 > 0: 
                        x_start = new_accel_last["x_start"] + new_accel_last["v_0"] * delta_t + 0.5 * new_accel_last["acc"] * delta_t**2
                    else:
                        # 減速区間の途中で速度が0になる場合. 
                        # 一旦ここは後で実装する。
                        raise ValueError("ブレーキの結果速度が0になりました")

                    segment_latter = {"t_start":new_t_end, "acc":current_acc_segment["acc"], "v_0":v_0, "x_start":x_start, "t_end":current_acc_segment["t_end"]}
                    new_itinerary.append(segment_latter) # b-2-3
                    insert_complete = True
                    continue
                else:
                    raise ValueError("NO SUCH CASE")
                                    

        # 新しいスケジュールで置き換え
        self._itinerary = new_itinerary
        
    
    def create_eta(self):
        return
        