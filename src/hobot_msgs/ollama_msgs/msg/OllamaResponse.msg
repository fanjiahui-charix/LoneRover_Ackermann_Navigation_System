# OllamaResponse.msg
#
# 用于描述从 Ollama 服务返回的 JSON 数据

# 消息发布时间戳
builtin_interfaces/Time timestamp

# 模型名称（例如 "llava"）
string model

# 创建时间（ISO8601 格式的字符串，或其他格式）
string created_at

# 模型的回答文本
string response

# 是否完成（例如 true 表示完成）
bool done

# 完成原因（例如 "stop"）
string done_reason

# 上下文信息，由整数数组表示
int32[] context

# 总时长（单位可根据实际情况调整，示例中为 uint64 类型）
uint64 total_duration

# 加载时长
uint64 load_duration

# prompt 评估次数
uint32 prompt_eval_count

# prompt 评估时长
uint64 prompt_eval_duration

# eval 评估次数
uint32 eval_count

# eval 评估时长
uint64 eval_duration
