# Alert.msg
# 患者ごとのアラート（異常検知/通知）イベント
#
# NOTE:
# - optional が無いので、未設定値は空文字 or NaN（float）で表現する
# - ts は UNIX time（秒）を想定（ROS time が必要なら builtin_interfaces/Time に置換する）

# 対象患者
string patient_id

# 安定ルールID（例: single.spo2_lt_90, roc.hr_jump, combo.spo2_drop_and_hr_jump）
string rule_id

# 判定種別（single/outlier/combination/rate_of_change/data_state など）
string kind

# 優先度（OK/INFO/YELLOW/ORANGE/RED/STALE/NO_DATA など）
string priority

# 人が読むためのメッセージ
string message

# 発火時刻（UNIX seconds）
float64 ts

# 評価窓（秒）。単発の場合は 0
uint32 window_sec

# 関連フィールド名（例: oxygen_saturation, heart_rate）。不要なら空文字
string field

# 関連値（不要なら NaN）
float32 value

# 変化量（不要なら NaN）
float32 delta

# スコア（不要なら NaN）
float32 score
