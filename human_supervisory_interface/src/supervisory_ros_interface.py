
#!/usr/bin/env python3

import uuid
from datetime import datetime

import streamlit as st
import rospy

from ros_backend import SupervisorBackend


# ============================================================
# CONFIGURAZIONE
# ============================================================

st.set_page_config(
    page_title="Robot Supervisor",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="collapsed",
)

REFRESH_SECONDS = 1.5
MAX_CAMERAS = 6
MAX_ACTIVITIES = 100


# STYLE

st.markdown(
    """
    <style>
        .block-container {
            padding-top: 1rem;
            padding-bottom: 1rem;
            max-width: 1800px;
        }

        [data-testid="stMetric"] {
            background: var(--secondary-background-color);
            padding: 12px 16px;
            border-radius: 10px;
            border: 1px solid rgba(128, 128, 128, 0.18);
        }

        div.stButton > button {
            border-radius: 8px;
            font-weight: 500;
        }
    </style>
    """,
    unsafe_allow_html=True,
)



# BACKEND ROS

@st.cache_resource
def get_backend():
    if not rospy.core.is_initialized():
        rospy.init_node(
            "supervisory_interface",
            anonymous=True,
            disable_signals=True,
        )

    return SupervisorBackend()


try:
    backend = get_backend()
except Exception as exc:
    st.error(f"ROS initialization impossible: {exc}")
    st.stop()


# SESSION STATE

DEFAULT_STATE = {
    "notice": None,
    "activities": [],
    "camera_topics": [],
}

for key, value in DEFAULT_STATE.items():
    if key not in st.session_state:
        st.session_state[key] = (
            value.copy()
            if isinstance(value, (list, dict))
            else value
        )


# COMMON FUNCTIONS

def notify(level, message):
    """Save the notification banner until next command."""
    st.session_state.notice = {
        "level": level,
        "message": message,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
    }


def log_activity(
    description,
    status,
    result=None,
    error=None,
    task_ids=None,
):
    """Save each command."""
    activity = {
        "id": str(uuid.uuid4()),
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "description": description,
        "status": status,
        "result": result,
        "error": error,
        "task_ids": list(task_ids or []),
    }

    st.session_state.activities.insert(0, activity)
    st.session_state.activities = (
        st.session_state.activities[:MAX_ACTIVITIES]
    )


def execute_command(description, callback, success_message):
    """
    Execute the command and save the result.

    'accepted' means the call hasn't generated exception.
    """
    try:
        result = callback()

        task_ids = (
            list(result or [])
            if isinstance(result, (list, tuple))
            else []
        )

        log_activity(
            description=description,
            status="accepted",
            result=result,
            task_ids=task_ids,
        )

        notify("success", success_message)
        return result

    except Exception as exc:
        log_activity(
            description=description,
            status="failed",
            error=str(exc),
        )

        notify("error", f"{description}: {exc}")
        return None


def get_task_status(task):
    return str(task.get("status", "unknown")).lower()



# HEADER

st.title("🤖 Robot Supervisor")
st.caption(
    f"World: {backend.world_id}"
)


# DYNAMIC BANNER

@st.fragment(run_every=REFRESH_SECONDS)
def render_notice():
    notice = st.session_state.notice

    if not notice:
        return

    message = (
        f"{notice['message']} · {notice['timestamp']}"
    )

    if notice["level"] == "success":
        st.success(f"✓ {message}")
    elif notice["level"] == "error":
        st.error(f"✕ {message}")
    elif notice["level"] == "warning":
        st.warning(f"⚠ {message}")
    else:
        st.info(message)


render_notice()


# DYNAMIC METRICS 

@st.fragment(run_every=REFRESH_SECONDS)
def render_metrics():
    try:
        tasks = backend.get_tasks() or {}
    except Exception as exc:
        st.warning(
            f"Updating metrics impossible: {exc}"
        )
        tasks = None

    if tasks is None:
        return

    statuses = [
        get_task_status(task)
        for task in tasks.values()
    ]

    active = sum(
        status in {
            "running", "pending", "queued",
            "in_progress", "accepted",
        }
        for status in statuses
    )

    completed = sum(
        status in {
            "completed", "complete", "success",
            "succeeded", "done",
        }
        for status in statuses
    )

    failed = sum(
        status in {"failed", "error", "aborted"}
        for status in statuses
    )

    cols = st.columns(5)

    cols[0].metric(
        "Robot configured",
        len(backend.robots_list),
    )
    cols[1].metric(
        "Cameras configured",
        len(backend.cameras_list),
    )
    cols[2].metric(
        "Doors / actuators",
        len(backend.actuators_list),
    )
    cols[3].metric("Active tasks", active)
    cols[4].metric("Failed tasks", failed)


render_metrics()

st.divider()


# DASHBOARD

robot_col, camera_col, operations_col = st.columns(
    [1.05, 1.65, 1.05],
    gap="large",
)


# ROBOT

@st.fragment
def render_robots():
    st.subheader("🤖 Robot")
    st.caption(
        "Choose robot to deploy and the final area to reach."
    )

    robots = backend.robots_list
    areas = backend.areas_list

    if not robots:
        st.info("No available robots.")
        return

    if not areas:
        st.warning("No available areas.")
        return

    # Il reset viene applicato prima di creare i widget.
    if st.session_state.pop(
        "reset_robot_selection", False
    ):
        st.session_state["select_all_robots"] = False

        for robot in robots:
            st.session_state.pop(f"deploy_{robot}", None)

    select_all = st.checkbox(
        "Select all robots",
        key="select_all_robots",
    )

    deployments = []

    for robot in robots:
        with st.container(border=True):
            selected = st.checkbox(
                robot,
                value=select_all,
                key=f"deploy_{robot}",
            )

            area = st.selectbox(
                "Destination",
                options=areas,
                key=f"area_{robot}",
                label_visibility="collapsed",
            )

            if selected:
                deployments.append({
                    "robot_to_send": robot,
                    "area_to_reach": area,
                })

    st.caption(
        f"{len(deployments)} robot selected"
    )

    if st.button(
        "🚀 Execute deployment",
        type="primary",
        use_container_width=True,
        disabled=not deployments,
        key="deploy_batch_button",
    ):
        batch = [
            deployment.copy()
            for deployment in deployments
        ]

        result = execute_command(
            description=(
                "Deployment: "
                + ", ".join(
                    f"{item['robot_to_send']} → "
                    f"{item['area_to_reach']}"
                    for item in batch
                )
            ),
            callback=lambda: backend.deploy_batch(batch),
            success_message=(
                f"Deployment request sent for "
                f"{len(batch)} robot."
            ),
        )

        if result is not None:
            st.session_state.reset_robot_selection = True


with robot_col:
    render_robots()



# ============================================================
# TELECAMERE
# ============================================================

@st.fragment
def render_cameras():
    st.subheader("📹 Cameras")
    st.caption("Handle camera feeds.")

    cameras = backend.cameras_list

    if not cameras:
        st.info("No cameras available.")
        return

    selected = st.multiselect(
        "Available cameras",
        options=cameras,
        max_selections=MAX_CAMERAS,
        key="camera_selection_widget",
        placeholder="Choose cameras",
    )

    open_col, close_col = st.columns(2)

    with open_col:
        if st.button(
            "▶ Open camera feed",
            type="primary",
            use_container_width=True,
            disabled=not selected,
            key="open_cameras_button",
        ):
            requested = list(selected)

            result = execute_command(
                description=(
                    "Opening camera: "
                    + ", ".join(requested)
                ),
                callback=lambda: backend.open_cameras(requested),
                success_message=(
                    f"Opening camera request sent for "
                    f"{len(requested)} telecamere."
                ),
            )

            if result is not None:
                if not isinstance(result, dict):
                    notify(
                        "error",
                        "Response not valid for open_cameras().",
                    )

                    log_activity(
                        description="Validation camera response",
                        status="failed",
                        error=(
                            "open_cameras() is not "
                            "a dictionary."
                        ),
                    )
                else:
                    topics = result.get("opened_topics", [])

                    st.session_state.camera_topics = list(
                        dict.fromkeys(
                            st.session_state.camera_topics
                            + topics
                        )
                    )

                    # Reset della selezione nel rerun successivo
                    # del fragment, prima di ricreare il widget.
                    st.session_state.reset_camera_selection = True

    with close_col:
        if st.button(
            "■ Close camera feed",
            use_container_width=True,
            disabled=not st.session_state.camera_topics,
            key="close_cameras_button",
        ):
            result = execute_command(
                description="Closing all camera feeds ",
                callback=backend.close_cameras,
                success_message=(
                    "Closing camera feed request sent."
                ),
            )

            if result is not None:
                st.session_state.camera_topics = []
                st.session_state.reset_camera_selection = True

    # Il reset deve precedere la creazione del multiselect.
    # Il flag viene quindi applicato al rerun successivo.
    if st.session_state.pop("reset_camera_selection", False):
        st.session_state.pop("camera_selection_widget", None)
        st.rerun(scope="fragment")

    st.divider()
    st.markdown("**Current feed required**")

    topics = st.session_state.camera_topics

    if topics:
        for topic in topics:
            st.code(str(topic), language=None)
    else:
        st.caption("No topics registred.")


with camera_col:
    render_cameras()


# ============================================================
# PORTE E ATTUATORI
# ============================================================

@st.fragment
def render_doors():
    st.subheader("🚪 Doors and actuators")
    st.caption("Send command to actuators.")

    actuators = backend.actuators_list

    if not actuators:
        st.info("No available actuators.")
        return

    selected_doors = st.multiselect(
        "Choose doors",
        options=actuators,
        key="selected_doors",
        placeholder="Choose one or more doors",
    )

    command = st.radio(
        "Command",
        options=["open", "close"],
        format_func=lambda value: (
            "Open" if value == "open" else "Close"
        ),
        horizontal=True,
        key="door_command",
    )

    if st.button(
        "Execute command",
        type="primary",
        use_container_width=True,
        disabled=not selected_doors,
        key="door_command_button",
    ):
        doors = list(selected_doors)

        commands = [
            {
                "actuator_name": door,
                "command": command,
            }
            for door in doors
        ]

        result = execute_command(
            description=(
                f"{'Opening' if command == 'open' else 'Closing'} "
                f"doors: {', '.join(doors)}"
            ),
            callback=lambda: backend.control_doors_batch(commands),
            success_message=(
                f"Sent commands "
                f"{'opening' if command == 'open' else 'closing'} "
                f"to {len(doors)} actuators."
            ),
        )

        if result is not None:
            st.session_state.reset_door_selection = True

    # Reset prima di ricreare i widget nel rerun successivo.
    if st.session_state.pop("reset_door_selection", False):
        st.session_state.pop("selected_doors", None)
        st.rerun(scope="fragment")


with operations_col:
    render_doors()


# RECENT ACTIVITIES


@st.fragment(run_every=2.0)
def render_recent_activities():
    st.subheader("📋 Recent Activities")

    try:
        tasks = backend.get_tasks() or {}
    except Exception as exc:
        st.error(f"Error retrieving tasks: {exc}")
        return

    if not tasks:
        st.caption("No activities available.")
        return

    # Display the 20 most recent tasks
    recent_tasks = list(tasks.items())[::-1][:20]

    for task_id, task in recent_tasks:
        description = task.get(
            "description",
            "Operation without description",
        )
        status = str(task.get("status", "unknown")).lower()

        with st.expander(
            f"{description} — {status}",
            expanded=False,
        ):
            st.caption(f"Task ID: {task_id}")
            st.write(f"Status: {status}")

            if task.get("result") is not None:
                st.write("Result")
                st.json(task["result"])

            if task.get("error"):
                st.error(str(task["error"]))


with operations_col:
    render_recent_activities()



# FOOTER

st.divider()

st.caption(
    "Robot Supervisor · ROS supervisory interface"
)
