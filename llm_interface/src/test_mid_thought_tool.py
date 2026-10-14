
import os
import json
from openai import OpenAI, OpenAIError


# ============================================================
# 1. OpenAI client
# ============================================================

api_key = os.getenv("OPENAI_API_KEY")

if not api_key:
    raise RuntimeError(
        "OPENAI_API_KEY is not set. Please configure your API key."
    )

client = OpenAI(api_key=api_key)


# ============================================================
# 2. Mock backend: distance calculation
# ============================================================

def compute_distances(target_area: str) -> str:
    """
    MOCK IMPLEMENTATION.

    Replace this function with the actual ROS make_plan
    service integration when the backend is ready.
    """

    print(
        f"\n[BACKEND] Running MOCK path planning "
        f"for target area: {target_area}"
    )

    # Predefined values for flow testing only.
    distances = [
        {
            "robot": "tb3_1",
            "path_length_meters": 5.2
        },
        {
            "robot": "tb3_2",
            "path_length_meters": 12.8
        }
    ]

    return json.dumps(distances)


# ============================================================
# 3. Tool schema
# ============================================================

tools = [
    {
        "type": "function",
        "function": {
            "name": "compute_distances",
            "description": (
                "Returns MOCK path distances for available robots "
                "to a specified target area. "
                "This demo does not execute ROS path planning."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target_area": {
                        "type": "string",
                        "description": (
                            "The area to investigate, "
                            "for example Area B."
                        )
                    }
                },
                "required": ["target_area"],
                "additionalProperties": False
            },
            "strict": True
        }
    }
]


# ============================================================
# 4. Mock alarm and context
# ============================================================

messages = [
    {
        "role": "system",
        "content": (
            "You are an LLM-based surveillance supervisor. "
            "The alarm and area context in this demo are MOCK data. "
            "Use the provided context to identify the target area. "
            "Briefly explain your target-area selection in one sentence, "
            "without revealing private chain-of-thought. "
            "Then call the compute_distances tool. "
            "Do not propose a deployment decision until the tool "
            "results have been received. "
            "After receiving the results, provide a deployment proposal "
            "based on the reported path distances. "
            "Treat shorter path distance as the primary selection "
            "criterion, and clearly state that this is a proposal, "
            "not an executed robot command. "
            "Do not claim that ROS was executed or that a robot moved."
        )
    },
    {
        "role": "user",
        "content": (
            "MOCK ALARM: Motion detected at Lidar_2. "
            "MOCK CONTEXT: Lidar_2 is located in Area B. "
            "Determine which area needs investigation."
        )
    }
]


# ============================================================
# 5. First API call: identify area and request the tool
# ============================================================

try:
    print("[SYSTEM] Sending mock alarm to the LLM...\n")

    response = client.chat.completions.create(
        model="gpt-4o",
        messages=messages,
        tools=tools,
        tool_choice="required"
    )

    response_message = response.choices[0].message

    # Show the brief explanation, if provided.
    if response_message.content:
        print(
            "[LLM] Target-area explanation:\n"
            f"{response_message.content}\n"
        )

    # Stop if the expected tool call is missing.
    if not response_message.tool_calls:
        raise RuntimeError(
            "The model did not return the required tool call."
        )

    # Keep the assistant's tool-call message in the conversation.
    messages.append(response_message)


    # ========================================================
    # 6. Execute requested backend tools
    # ========================================================

    for tool_call in response_message.tool_calls:

        if tool_call.function.name != "compute_distances":
            raise RuntimeError(
                f"Unexpected tool: {tool_call.function.name}"
            )

        args = json.loads(tool_call.function.arguments)

        target_area = args.get("target_area")

        if not isinstance(target_area, str) or not target_area.strip():
            raise ValueError("Invalid target_area argument.")

        # Execute the Python backend function.
        distance_data = compute_distances(target_area)

        print(
            "[BACKEND] Returning MOCK distance data:\n"
            f"{json.dumps(json.loads(distance_data), indent=2)}\n"
        )

        # Return the tool result to the LLM.
        messages.append(
            {
                "role": "tool",
                "tool_call_id": tool_call.id,
                "content": distance_data
            }
        )


    # ========================================================
    # 7. Second API call: final deployment proposal
    # ========================================================

    print(
        "[SYSTEM] Sending tool results back to the LLM "
        "for the final proposal...\n"
    )

    final_response = client.chat.completions.create(
        model="gpt-4o",
        messages=messages,
        tools=tools,
        tool_choice="none"
    )

    final_message = final_response.choices[0].message

    if final_message.content:
        print(
            "[LLM FINAL DEPLOYMENT PROPOSAL]\n"
            f"{final_message.content}"
        )
    else:
        print("[LLM] No final text response was returned.")


except (OpenAIError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
    print(f"[ERROR] Flow execution failed: {exc}")
