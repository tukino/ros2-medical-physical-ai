# Day3: shutdown / clean exit（rclpy）メモ

## 背景
ROS 2 の launch 配下で複数ノードを起動し、`Ctrl+C`（SIGINT）で停止した際に、Python ノード側で例外スタックトレースが出たり、終了コードが 0 にならないことがありました。

Day3 では、停止シーケンスを「正常系」として扱い、**例外を出さずにクリーンに終了する**こと（= public repo としての再現性・安心感）を優先して修正しました。

## 症状（起きがちなもの）
- `rclpy.spin()` 中に `ExternalShutdownException` が上がる
- `finally` などで `rclpy.shutdown()` を呼ぶと、状況によっては
  - `rcl_shutdown already called on the given context` のような RCLError が出る

ポイントは、launch/シグナル/他ノードの終了順によって **shutdown が既に呼ばれているケースがある**ことです。

## 原因（よくあるパターン）
- `Ctrl+C` などで executor が外部 shutdown と判断し、`ExternalShutdownException` を投げる
- その後、ユーザコード側がさらに `rclpy.shutdown()` を呼んで **二重 shutdown** になり得る

## 対応（推奨パターン）
- `ExternalShutdownException` / `KeyboardInterrupt` を「正常終了」とみなす
- `finally` で後片付けを行う
  - `destroy_node()` は例外が出ることがあるので安全側に
  - `rclpy.shutdown()` は `rclpy.ok()` を見てから実行（= 二重 shutdown 回避）

最小形（概念）:

```python
from rclpy.executors import ExternalShutdownException

try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
```

## 確認手順（再現性チェック）
- `ros2 launch ...` で起動 → `Ctrl+C` で停止
  - スタックトレースが出ない
  - 終了後に `ros2 node list` / `ps` でプロセスが残っていない

例:
```bash
# 起動
ros2 launch medical_robot_sim icu_multi_patient.launch.py patients:=patient_01,patient_02

# 停止（Ctrl+C）後の簡易チェック
ros2 node list
pgrep -af medical_robot_sim
```

## 補足
- launch のログで「process has died」と出ても、別ターミナルで `ps` を見ると残っているように見えることがあります。
  - PID の取り違えや、表示タイミング（終了直後の残像）などでも起こり得るため、`pgrep -af` でコマンドライン（`__ns:=...` など）まで含めて確認すると確実です。
