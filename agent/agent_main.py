"""Agent：用 LLM 决策，通过 MCP 控仿真，通过 ZMQ 感知 LiDAR 数据流。"""
import asyncio
import json
import os
import re

import zmq
from mcp import Client
from mcp.client.streamable_http import streamable_http_client
from openai import OpenAI

LLM_BASE = os.environ.get("LLM_BASE", "http://host.docker.internal:8080/v1")
LLM_MODEL = os.environ.get("LLM_MODEL", "Qwen3.5-4B_Q4_K_M.gguf")
MCP_URL = os.environ.get("MCP_URL", "http://mujoco-wrapper:8081/mcp")
ZMQ_ENDPOINT = os.environ.get("ZMQ_ENDPOINT", "tcp://mujoco-wrapper:5555")


def read_lidar_once(timeout_ms=5000) -> dict:
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(ZMQ_ENDPOINT)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, timeout_ms)
    try:
        raw = sub.recv()
        return {"received_bytes": len(raw)}
    except zmq.Again:
        return {"error": "timeout"}
    finally:
        sub.close()
        ctx.term()


def extract_json(text: str):
    if not text:
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(1))
        except json.JSONDecodeError:
            pass
    m = re.search(r"\{.*\}", text, re.DOTALL)
    if m:
        try:
            return json.loads(m.group(0))
        except json.JSONDecodeError:
            pass
    return None


async def run_agent(client, llm):
    tools = await client.list_tools()
    print(f"[agent] MCP tools: {[t.name for t in tools.tools]}")

    lidar = read_lidar_once()
    print(f"[agent] lidar: {lidar}")

    prompt = (
        f"你是机器人控制 Agent。当前 LiDAR 统计：{json.dumps(lidar)}。"
        "可用 MCP 工具：set_base_velocity(vx, omega)、move_joint(actuator_name, pos)、get_lidar_snapshot()。"
        "如果点云非空，先调 get_lidar_snapshot 确认；然后可以调 set_base_velocity(0.1, 0) 让底盘前进。"
        "只输出一个 JSON：{\"tool\": \"<name>\", \"arguments\": {}}。不要输出其他内容。"
    )

    resp = llm.chat.completions.create(
        model=LLM_MODEL,
        messages=[{"role": "user", "content": prompt}],
        max_tokens=512,
        extra_body={"chat_template_kwargs": {"enable_thinking": False}},
    )

    msg = resp.choices[0].message
    content = (msg.content or "").strip()
    print(f"[agent] LLM content={content!r}")

    call = extract_json(content)
    if call is None:
        print("[agent] no JSON in LLM output, skip tool call")
        return

    try:
        tool_name = call["tool"]
        args = call.get("arguments", {})
        result = await client.call_tool(tool_name, args)
        print(f"[agent] tool result: {result}")
    except Exception as e:
        print(f"[agent] parse/exec failed: {e}")


async def main():
    llm = OpenAI(base_url=LLM_BASE, api_key="dummy")

    success = False
    for attempt in range(60):
        try:
            async with Client(streamable_http_client(MCP_URL)) as client:
                await run_agent(client, llm)
                success = True
        except Exception as e:
            print(f"[agent] MCP not ready ({attempt+1}/60): {type(e).__name__}: {e}")
        if success:
            break
        await asyncio.sleep(2.0)

    if not success:
        raise RuntimeError("MCP not ready after 2 minutes")


if __name__ == "__main__":
    asyncio.run(main())