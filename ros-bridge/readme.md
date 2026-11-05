proto 的 JSON 头里只放 topic，payload 是完整的 Protobuf 消息。
ROS bridge 侧用 proto 反序列化，Agent 侧不参与。
