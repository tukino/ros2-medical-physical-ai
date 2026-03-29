# BCIFeatures.msg
# EEG/BCI 由来の特徴量（features）を最小セットで運ぶ
#
# NOTE:
# - optional が無いので、未設定値は空文字 or NaN（float）で表現する
# - 生波形ではなく、下流が扱いやすい特徴量のみに絞る

# 対象患者
string patient_id

# 単調増加の測定ID（deterministic な再現や順序確認用）
uint32 measurement_id

# 特徴量（0..1 を想定。欠測は NaN）
float32 attention
float32 drowsiness
float32 signal_quality

# デバイス状態（例: ok/degraded/no_signal）
string status
