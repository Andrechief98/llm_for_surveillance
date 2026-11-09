
#!/usr/bin/env python3

import uuid
import logging

from datetime import datetime

import streamlit as st
import rospy

from ros_backend import SupervisorBackend


# ============================================================
# CONFIGURATION
# ============================================================

st.set_page_config(
    page_title="Robot Supervisor",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="collapsed",
)

MAX_CAMERAS = 6
MAX_ACTIVITIES = 100

GOAL_REFRESH = 2.0
SENSOR_REFRESH = 5.0
TASK_REFRESH = 3.0

logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)


# ============================================================
# STYLE
# ============================================================

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
            border: 1px solid rgba(128,128,128,0.18);
        }

        div.stButton > button {
            border-radius: 8px;
            font-weight: 500;
        }
    </style>
    """,
    unsafe_allow_html=True,
)


# ============================================================
# BACKEND INITIALIZATION
# ============================================================

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

    logger.exception("Backend initialization failed")

    st.error(
        f"Unable to initialize ROS backend: {exc}"
    )

    st.stop()


# ============================================================
# SESSION STATE
# ============================================================

DEFAULT_STATE = {
    "notice": None,
    "activities": [],
}

for key, value in DEFAULT_STATE.items():

    if key not in st.session_state:

        st.session_state[key] = (
            value.copy()
            if isinstance(value, (list, dict))
            else value
        )


# ============================================================
# COMMON FUNCTIONS
# ============================================================

def notify(level, message):

    st.session_state.notice = {
        "level": level,
        "message": message,
        "timestamp": datetime.now().strftime("%H:%M:%S"),
    }


def log_activity(description, status, result=None, error=None):

    activity = {
        "id": str(uuid.uuid4()),
        "timestamp": datetime.now().strftime("%H:%M:%S"),
        "description": description,
        "status": status,
        "result": result,
        "error": error,
    }

    activities = st.session_state.activities

    activities.insert(0, activity)

    st.session_state.activities = (
        activities[:MAX_ACTIVITIES]
    )


def execute_command(description, callback, success_message):

    try:

        result = callback()

        log_activity(
            description=description,
            status="accepted",
            result=result,
        )

        notify(
            "success",
            success_message,
        )

        return result

    except Exception as exc:

        logger.exception(
            "Command failed: %s",
            description,
        )

        log_activity(
            description=description,
            status="failed",
            error=str(exc),
        )

        notify(
            "error",
            f"{description}: {exc}",
        )

        return None


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


# ============================================================
# HEADER
# ============================================================

st.title("🤖 Robot Supervisor")

st.caption(
    f"World: {backend.world_id}"
)

render_notice()




# ============================================================
# ROBOT GOAL VERIFICATION
# ============================================================

@st.fragment(run_every=1.0)
def render_robot_goal_events():

    st.subheader(
        "🎯 Goal Verification & Intruder Detection"
    )

    try:

        events = backend.get_robot_goal_events() or {}

    except Exception as exc:

        st.error(
            f"Unable to retrieve robot events: {exc}"
        )

        return

    if not events:

        st.info(
            "No robot verification results received yet."
        )

        return

    for robot, event in events.items():

        goal_reached = event.get("goal_reached")

        intruder_detected = event.get(
            "intruder_detected"
        )

        with st.container(border=True):

            st.markdown(
                f"### 🤖 {event.get('robot', robot)}"
            )

            st.caption(
                f"Destination: {event.get('area', 'unknown')} "
                f"· Updated: {event.get('timestamp', 'unknown')}"
            )

            goal_col, detection_col = st.columns(2)

            with goal_col:

                st.markdown("**Navigation status**")

                if goal_reached is True:

                    st.success("✓ Goal reached")

                elif goal_reached is False:

                    st.error("✕ Goal not reached")

                else:

                    st.warning(
                        "Navigation status unavailable"
                    )

            with detection_col:

                st.markdown("**Intruder detection**")

                if goal_reached is not True:

                    st.info(
                        "Detection not completed"
                    )

                elif intruder_detected is True:

                    st.error("⚠ Intruder detected")

                elif intruder_detected is False:

                    st.success("✓ No intruder detected")

                else:

                    st.warning(
                        "Detection unavailable"
                    )

            message = event.get("message", "")

            if message:

                st.caption(
                    f"Details: {message}"
                )

            if event.get("status") == "failed":

                if event.get("goal_status") == "Verification error":

                    st.error(
                        f"Verification error: {message}"
                    )


render_robot_goal_events()



# ============================================================
# ACTIVE SENSORS
# ============================================================

@st.fragment(run_every=1.0)
def render_active_sensors():

    try:
        data = backend.get_active_sensors()
        sensors = data.get("sensors", [])

    except Exception:
        st.caption("📡 Sensors unavailable")
        return

    if sensors:
        st.success(
            "📡 Active sensors: " + ", ".join(map(str, sensors))
        )
    else:
        st.caption("📡 No active sensors")


render_active_sensors()

st.divider()



# ============================================================
# EXPERIMENT CONTROL
# ============================================================

st.subheader("🧪 Experiment Control")

st.caption(
    "Control the experiment and report inspection results."
)

col1, col2, col3, col4 = st.columns(4, gap="medium")


# ------------------------------------------------------------
# NEXT SENSOR SEQUENCE
# ------------------------------------------------------------

with col1:

    if st.button(
        "⏭️ Next sensor sequence",
        type="primary",
        use_container_width=True,
        key="next_sequence",
    ):

        execute_command(
            "Advance sensor sequence",
            backend.next_sensor_sequence,
            "Next sensor sequence requested.",
        )


# ------------------------------------------------------------
# STOP EXPERIMENT
# ------------------------------------------------------------

with col2:

    if st.button(
        "🛑 Stop experiment",
        type="secondary",
        use_container_width=True,
        key="stop_experiment",
    ):

        execute_command(
            "Stop experiment",
            backend.stop_experiment,
            "Experiment stop requested.",
        )


# ------------------------------------------------------------
# INTRUDER DETECTED
# ------------------------------------------------------------

with col3:

    if st.button(
        "🚨 Intruder detected",
        type="primary",
        use_container_width=True,
        key="intruder_detected",
    ):

        execute_command(
            "Intruder detected",
            backend.report_intruder_detected,
            "Intruder detection reported.",
        )


# ------------------------------------------------------------
# FALSE POSITIVE
# ------------------------------------------------------------

with col4:

    if st.button(
        "✓ False positive",
        type="secondary",
        use_container_width=True,
        key="false_positive",
    ):

        execute_command(
            "False positive",
            backend.report_false_positive,
            "False positive reported.",
        )


st.divider()


# ============================================================
# MAIN DASHBOARD
# ============================================================

robot_col, camera_col, operations_col = st.columns(
    [1.05, 1.65, 1.05],
    gap="large",
)


# ============================================================
# ROBOT CONTROLS
# ============================================================

with robot_col:

    st.subheader("🤖 Robot")

    st.caption(
        "Choose robots and their destination areas."
    )

    robots = backend.robots_list
    areas = backend.areas_list

    if not robots:

        st.info("No available robots.")

    elif not areas:

        st.warning("No available areas.")

    else:

        def update_all_robots():

            selected = st.session_state.select_all_robots

            for robot in backend.robots_list:

                st.session_state[f"deploy_{robot}"] = selected


        st.checkbox(
            "Select all robots",
            key="select_all_robots",
            on_change=update_all_robots,
        )

        deployments = []

        for robot in robots:

            with st.container(border=True):

                selected = st.checkbox(
                    robot,
                    key=f"deploy_{robot}",
                )

                area = st.selectbox(
                    f"Destination: {robot}",
                    options=areas,
                    key=f"area_{robot}",
                )

                if selected:

                    deployments.append({
                        "robot_to_send": robot,
                        "area_to_reach": area,
                    })

        st.caption(
            f"{len(deployments)} robots selected"
        )

        if st.button(
            "🚀 Execute deployment",
            type="primary",
            use_container_width=True,
            disabled=not deployments,
            key="deploy_batch",
        ):

            batch = [
                item.copy()
                for item in deployments
            ]

            result = execute_command(
                "Robot deployment",
                lambda: backend.deploy_batch(batch),
                f"Deployment submitted for {len(batch)} robots.",
            )

            if result is not None:

                st.session_state.select_all_robots = False

                for robot in robots:

                    st.session_state[f"deploy_{robot}"] = False


# ============================================================
# CAMERA CONTROLS
# ============================================================

with camera_col:

    st.subheader("📹 Cameras")

    st.caption(
        "Manage camera feeds."
    )

    cameras = backend.cameras_list

    if not cameras:

        st.info("No cameras available.")

    else:

        selected_cameras = st.multiselect(
            "Available cameras",
            options=cameras,
            max_selections=MAX_CAMERAS,
            key="camera_selection",
            placeholder="Choose cameras",
        )

        open_col, close_col = st.columns(2)

        with open_col:

            if st.button(
                "▶ Open camera feed",
                type="primary",
                use_container_width=True,
                disabled=not selected_cameras,
                key="open_camera",
            ):

                requested = list(selected_cameras)

                result = execute_command(
                    "Open camera feeds",
                    lambda: backend.submit_task(
                        "Open cameras",
                        backend.open_cameras,
                        requested,
                    ),
                    f"Camera opening submitted for {len(requested)} cameras.",
                )

                if result is not None:

                    st.session_state.camera_selection = []


        with close_col:

            if st.button(
                "■ Close camera feed",
                use_container_width=True,
                key="close_camera",
            ):

                execute_command(
                    "Close camera feeds",
                    lambda: backend.submit_task(
                        "Close cameras",
                        backend.close_cameras,
                    ),
                    "Camera closing submitted.",
                )

    st.divider()

    st.markdown("**Current active feeds**")

    topics = backend.get_camera_topics()

    if topics:

        for topic in topics:

            st.code(
                str(topic),
                language=None,
            )

    else:

        st.caption(
            "No active camera feeds."
        )


# ============================================================
# DOOR AND ACTUATOR CONTROLS
# ============================================================

with operations_col:

    st.subheader("🚪 Doors and actuators")

    st.caption(
        "Send commands to actuators."
    )

    actuators = backend.actuators_list

    if not actuators:

        st.info("No available actuators.")

    else:

        selected_doors = st.multiselect(
            "Choose doors",
            options=actuators,
            key="selected_doors",
            placeholder="Choose one or more doors",
        )

        door_command = st.radio(
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
            key="execute_doors",
        ):

            commands = [
                {
                    "actuator_name": door,
                    "command": door_command,
                }
                for door in selected_doors
            ]

            result = execute_command(
                "Door control",
                lambda: backend.control_doors_batch(commands),
                f"Commands submitted to {len(commands)} actuators.",
            )

            if result is not None:

                st.session_state.selected_doors = []



# ============================================================
# GLOBAL EXECUTION
# ============================================================

st.divider()

# st.subheader("🚀 Global Execution")

# st.caption(
#     "Execute all selected robot, camera and door operations."
# )


# ------------------------------------------------------------
# COLLECT ROBOT DEPLOYMENTS
# ------------------------------------------------------------

deployments = []

for robot in backend.robots_list:

    if st.session_state.get(
        f"deploy_{robot}",
        False,
    ):

        area = st.session_state.get(
            f"area_{robot}",
        )

        if area:

            deployments.append({
                "robot_to_send": robot,
                "area_to_reach": area,
            })


# ------------------------------------------------------------
# COLLECT CAMERAS
# ------------------------------------------------------------

selected_cameras = st.session_state.get(
    "camera_selection",
    [],
)


# ------------------------------------------------------------
# COLLECT DOORS
# ------------------------------------------------------------

selected_doors = st.session_state.get(
    "selected_doors",
    [],
)

door_command = st.session_state.get(
    "door_command",
    "open",
)

door_commands = [
    {
        "actuator_name": door,
        "command": door_command,
    }
    for door in selected_doors
]


# ------------------------------------------------------------
# SUMMARY
# ------------------------------------------------------------

total_operations = (
    len(deployments)
    + (1 if selected_cameras else 0)
    + len(door_commands)
)

# summary_col1, summary_col2, summary_col3 = st.columns(3)

# with summary_col1:

#     st.metric(
#         "Robot deployments",
#         len(deployments),
#     )

# with summary_col2:

#     st.metric(
#         "Camera groups",
#         1 if selected_cameras else 0,
#     )

# with summary_col3:

#     st.metric(
#         "Door operations",
#         len(door_commands),
#     )


# ------------------------------------------------------------
# EXECUTE ALL
# ------------------------------------------------------------

if st.button(
    "🚀 EXECUTE ALL SELECTED ACTIONS",
    type="primary",
    use_container_width=True,
    disabled=(total_operations == 0),
    key="execute_all",
):

    result = execute_command(
        description="Global execution",

        callback=lambda: backend.execute_all_batch(
            deployments=[
                item.copy()
                for item in deployments
            ],

            camera_names=list(selected_cameras),

            door_commands=[
                item.copy()
                for item in door_commands
            ],
        ),

        success_message=(
            f"Global execution submitted: "
            f"{total_operations} operations."
        ),
    )


# ============================================================
# RECENT ACTIVITIES
# ============================================================

# st.divider()


# @st.fragment(run_every=TASK_REFRESH)
# def render_recent_activities():

#     st.subheader("📋 Recent Activities")

#     try:

#         tasks = backend.get_recent_tasks(limit=20)

#     except Exception as exc:

#         st.error(
#             f"Unable to retrieve activities: {exc}"
#         )

#         return

#     if not tasks:

#         st.caption(
#             "No activities available."
#         )

#         return

#     for task_id, task in tasks:

#         description = task.get(
#             "description",
#             "Operation without description",
#         )

#         status = str(
#             task.get("status", "unknown")
#         ).lower()

#         with st.expander(
#             f"{description} — {status}",
#             expanded=False,
#         ):

#             st.caption(
#                 f"Task ID: {task_id}"
#             )

#             st.write(
#                 f"Status: {status}"
#             )

#             if task.get("queue_time") is not None:

#                 st.caption(
#                     f"Queue time: {task['queue_time']:.3f} s"
#                 )

#             if task.get("execution_time") is not None:

#                 st.caption(
#                     f"Execution time: {task['execution_time']:.3f} s"
#                 )

#             if task.get("result") is not None:

#                 st.write("Result")

#                 st.json(
#                     task["result"]
#                 )

#             if task.get("error"):

#                 st.error(
#                     str(task["error"])
#                 )


# render_recent_activities()


# ============================================================
# FRONTEND ACTIVITY HISTORY
# ============================================================

with st.expander(
    "Frontend command history",
    expanded=False,
):

    activities = st.session_state.activities

    if not activities:

        st.caption("No commands submitted.")

    else:

        for activity in activities[:20]:

            st.write(
                f"**{activity['timestamp']}** — "
                f"{activity['description']} "
                f"({activity['status']})"
            )

            if activity["error"]:

                st.error(
                    activity["error"]
                )


# ============================================================
# FOOTER
# ============================================================

st.divider()

st.caption(
    "Robot Supervisor · ROS supervisory interface"
)
