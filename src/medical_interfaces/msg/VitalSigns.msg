# VitalSigns.msg
# 患者のバイタルサイン（生体情報）を表すメッセージ型
# 医療機器標準に準拠した設計

# 患者識別情報
string patient_id
uint32 measurement_id

# バイタルサイン測定値
uint16 heart_rate              # 心拍数 [bpm]
uint16 blood_pressure_systolic # 収縮期血圧 [mmHg]
uint16 blood_pressure_diastolic # 拡張期血圧 [mmHg]
float32 body_temperature       # 体温 [°C]
uint8 oxygen_saturation        # 酸素飽和度 [%]

# ステータス情報
string status                  # monitoring, alert, emergency