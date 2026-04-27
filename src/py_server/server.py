import threading
from threading import Lock
import uuid

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient

from ur_action_interface.action import MovePiece


# ----------------------------
# FastAPI setup
# ----------------------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten later
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ----------------------------
# Request models
# ----------------------------
class Slot(BaseModel):
    m: int
    r: int
    c: int

class MovePieceReq(BaseModel):
    piece_id: str
    from_: Slot = Field(..., alias="from")
    to: Slot
    speed: float = 0.3

    class Config:
        populate_by_name = True


# ----------------------------
# Shared robot and board state
# ----------------------------
_state_lock = Lock()

START_PIECES = [
    {"id": "A", "m": 1, "r": 1, "c": 1},
    {"id": "B", "m": 1, "r": 1, "c": 2},
    {"id": "C", "m": 2, "r": 1, "c": 1},
    {"id": "D", "m": 2, "r": 1, "c": 2},
]

ROBOT_STATE = {
    "state": "idle",          # idle | busy | error
    "current_job": None,      # str job_id
    "current_phase": None,    # from ROS feedback
    "progress": None,         # float 0..1
    "last_result": None,      # dict
    "last_error": None,       # str

    #board related
    "present_board": START_PIECES,   # confirmed
    "target_board": START_PIECES,    # future/desired (differs only while busy)
    "pending_move": None,            # {job_id, pieceId, from, to}
}

def _set_state(**kwargs):
    with _state_lock:
        ROBOT_STATE.update(kwargs)

def _get_state_copy():
    with _state_lock:
        return dict(ROBOT_STATE)
    
def _find_piece(board, piece_id: str):
    for p in board:
        if p["id"] == piece_id:
            return p
    return None

def _cell_eq(a, b):
    return a["m"] == b["m"] and a["r"] == b["r"] and a["c"] == b["c"]

def _is_occupied(board, cell, ignore_piece_id=None):
    for p in board:
        if ignore_piece_id and p["id"] == ignore_piece_id:
            continue
        if p["m"] == cell["m"] and p["r"] == cell["r"] and p["c"] == cell["c"]:
            return True
    return False

def _apply_move(board, piece_id: str, to_cell):
    new_board = [dict(p) for p in board]
    p = _find_piece(new_board, piece_id)
    if not p:
        raise ValueError(f"Unknown piece '{piece_id}'")
    p["m"], p["r"], p["c"] = int(to_cell["m"]), int(to_cell["r"]), int(to_cell["c"])
    return new_board


# ----------------------------
# ROS2 Action Client Node
# ----------------------------
class RosBridge(Node):
    def __init__(self):
        super().__init__("web_action_client")
        self.client = ActionClient(self, MovePiece, "move_piece")
        self.get_logger().info("ROS bridge node created (ActionClient: /move_piece)")

    def send_move_piece_goal(self, job_id: str, req: MovePieceReq):
        # Ensure server exists
        if not self.client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("Action server /move_piece not available")

        goal_msg = MovePiece.Goal()
        goal_msg.from_m = int(req.from_.m - 1)
        goal_msg.from_r = int(req.from_.r - 1)
        goal_msg.from_c = int(req.from_.c - 1)
        goal_msg.to_m   = int(req.to.m - 1)
        goal_msg.to_r   = int(req.to.r - 1)
        goal_msg.to_c   = int(req.to.c - 1)
        goal_msg.speed  = float(req.speed)

        # Send goal async with feedback
        send_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self._on_feedback(job_id, fb),
        )
        send_future.add_done_callback(lambda f: self._on_goal_response(job_id, f))

    def _on_goal_response(self, job_id: str, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            _set_state(state="error", current_job=None, current_phase=None, progress=None, last_error=str(e))
            return

        if not goal_handle.accepted:
            _set_state(
                state="idle",
                current_job=None,
                current_phase=None,
                progress=None,
                last_result={"job_id": job_id, "ok": False, "message": "Goal rejected"},
                last_error=None,
            )
            return

        # Goal accepted then wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._on_result(job_id, f))

    def _on_feedback(self, job_id: str, feedback_msg):
        # feedback_msg is a "GoalHandleFeedback" wrapper: feedback_msg.feedback is the actual feedback
        fb = feedback_msg.feedback
        st = _get_state_copy()

        # Only update if this feedback belongs to the current job
        if st["current_job"] != job_id:
            return

        _set_state(
            current_phase=str(fb.phase),
            progress=float(fb.progress),
        )

    def _on_result(self, job_id: str, future):
        try:
            result = future.result().result
        except Exception as e:
            _set_state(state="error", current_job=None, current_phase=None, progress=None, last_error=str(e))
            return

        ok = bool(result.success)
        msg = str(result.message)

        with _state_lock:
            st = dict(ROBOT_STATE)
            # only finalize if it’s still the same job
            if st["current_job"] != job_id:
                return

            if ok:
                ROBOT_STATE["present_board"] = st["target_board"]
            else:
                ROBOT_STATE["target_board"] = st["present_board"]

            ROBOT_STATE.update(
                state="idle",
                current_job=None,
                current_phase=None,
                progress=None,
                last_result={"job_id": job_id, "ok": ok, "message": msg},
                last_error=None if ok else msg,
                pending_move=None,
            )

# ----------------------------
# Start ROS in background thread once
# ----------------------------
_ros_lock = Lock()
_ros_started = False
_ros_node = None
_ros_exec = None
_ros_thread = None

def start_ros_once():
    global _ros_started, _ros_node, _ros_exec, _ros_thread
    with _ros_lock:
        if _ros_started:
            return

        rclpy.init(args=None)
        _ros_node = RosBridge()
        _ros_exec = SingleThreadedExecutor()
        _ros_exec.add_node(_ros_node)

        def spin():
            try:
                _ros_exec.spin()
            finally:
                try:
                    _ros_exec.shutdown()
                except Exception:
                    pass
                try:
                    _ros_node.destroy_node()
                except Exception:
                    pass
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

        _ros_thread = threading.Thread(target=spin, daemon=True)
        _ros_thread.start()
        _ros_started = True


@app.on_event("startup")
def _startup():
    # Start ROS bridge when FastAPI starts
    start_ros_once()


# ----------------------------
# API endpoints
# ----------------------------
@app.get("/api/status")
def status():
    return _get_state_copy()


@app.post("/api/move_piece")
def move_piece(req: MovePieceReq):
    st = _get_state_copy()
    if st["state"] == "busy":
        raise HTTPException(status_code=409, detail=f"Robot busy (job={st['current_job']})")

    job_id = uuid.uuid4().hex[:10]

    # Atomically validate + set pending + set target board
    with _state_lock:
        present = ROBOT_STATE["present_board"]

        # Validate piece exists and from matches what server believes
        p = _find_piece(present, req.piece_id)
        if not p:
            raise HTTPException(status_code=400, detail=f"Unknown piece_id '{req.piece_id}'")

        server_from = {"m": p["m"], "r": p["r"], "c": p["c"]}
        client_from = {"m": req.from_.m, "r": req.from_.r, "c": req.from_.c}
        if server_from != client_from:
            raise HTTPException(
                status_code=409,
                detail=f"Desync: server has {req.piece_id} at {server_from}, client said {client_from}",
            )

        to_cell = {"m": req.to.m, "r": req.to.r, "c": req.to.c}
        if _is_occupied(present, to_cell, ignore_piece_id=req.piece_id):
            raise HTTPException(status_code=409, detail="Target cell occupied")

        target = _apply_move(present, req.piece_id, to_cell)

        ROBOT_STATE.update(
            state="busy",
            current_job=job_id,
            current_phase="accepted",
            progress=0.0,
            last_error=None,
            pending_move={
                "job_id": job_id,
                "pieceId": req.piece_id,
                "from": server_from,
                "to": to_cell,
            },
            target_board=target,
        )

    # Send ROS goal
    try:
        _ros_node.send_move_piece_goal(job_id, req)
    except Exception as e:
        _set_state(
            state="error",
            current_job=None,
            current_phase=None,
            progress=None,
            last_error=str(e),
            pending_move=None,
            target_board=_get_state_copy()["present_board"],  # revert target
        )
        raise HTTPException(status_code=503, detail=str(e))

    return {"ok": True, "message": "job accepted", "job_id": job_id}
