import os
import sys
import time
import json
import math
import asyncio
from datetime import datetime
from typing import Optional, List

import numpy as np
import roslibpy
import uvicorn

import crud, models, schemas
from databases import SessionLocal, engine, Base, get_db

from fastapi import (
    FastAPI, Depends, HTTPException, status,
    Request, Form, WebSocket, WebSocketDisconnect
)
from fastapi.responses import (
    HTMLResponse, RedirectResponse, JSONResponse
)
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from sqlalchemy import text
from dotenv import load_dotenv


# ============================================================
# 1. 환경 변수 및 FastAPI 기본 설정
# ============================================================

load_dotenv()

app = FastAPI(title="Patrol Server")

# 정적 파일 및 템플릿
app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="static")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================
# 2. DB 초기화 (databases.py 기준)
# ============================================================

try:
    # models 모듈이 import 되면 User 등 모든 테이블이 Base에 등록된다
    Base.metadata.create_all(bind=engine)
    print("[DB] 테이블 생성 또는 확인 완료")
except Exception as e:
    print(f"[DB ERROR] 데이터베이스 초기화 실패: {e}")


# ============================================================
# 3. 로그인 유틸 및 초기 관리자 계정 생성
# ============================================================

async def get_current_user_optional(
    request: Request,
    db: Session = Depends(get_db)
) -> Optional[models.User]:
    """쿠키에서 사용자 ID 읽어서 User 객체 반환, 없으면 None."""
    user_id_str = request.cookies.get("user_id")
    if not user_id_str:
        return None

    try:
        user_id = int(user_id_str)
        user = crud.get_user(db, user_id=user_id)
        if user and user.status != "deleted":
            return user
    except Exception as e:
        print(f"[ERROR] Failed to retrieve user from cookie: {e}")
        return None

    return None


@app.on_event("startup")
def create_initial_admin_user():
    """서버 시작 시 초기 관리자 계정 생성 (이미 있으면 건너뜀)."""
    TARGET_EMPLOYEE_NUMBER = "E001"
    TARGET_PASSWORD = "adminpass"

    try:
        db = SessionLocal()
        existing_user = crud.get_user_by_employee(db, TARGET_EMPLOYEE_NUMBER)

        if existing_user is None:
            admin_data = schemas.UserCreate(
                employee_number=TARGET_EMPLOYEE_NUMBER,
                username="inho",
                password=TARGET_PASSWORD,
                name="inho",
                role="admin",
            )
            crud.create_user(db, admin_data)
            print(
                f"[INITIAL SETUP] 초기 관리자 계정 생성 완료 "
                f"(사번: {TARGET_EMPLOYEE_NUMBER}, PW: {TARGET_PASSWORD})"
            )
        else:
            print(
                f"[INITIAL SETUP] 관리자 계정(사번: {TARGET_EMPLOYEE_NUMBER}) 이미 존재"
            )
        db.close()
    except Exception as e:
        print(f"[INITIAL SETUP ERROR] 초기 설정 실패: {e}")


@app.get("/health")
def health():
    """DB 연결 상태 확인용."""
    try:
        with engine.connect() as conn:
            conn.execute(text("SELECT 1"))
        return {"status": "ok", "db": "connected"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Database connection error: {e}",
        )


# ============================================================
# 4. HTML 화면 라우터
# ============================================================

@app.get("/", response_class=HTMLResponse)
async def root_page(
    request: Request,
    current_user: Optional[models.User] = Depends(get_current_user_optional),
):
    """루트: 로그인 페이지 또는 메인으로 리다이렉트."""
    if current_user:
        if current_user.role == "admin":
            return RedirectResponse(url="/main", status_code=status.HTTP_303_SEE_OTHER)
        else:
            return RedirectResponse(
                url="/main_user", status_code=status.HTTP_303_SEE_OTHER
            )

    error_message = request.query_params.get("error")
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "error_message": error_message,
            "title": "로그인",
        },
    )


@app.get("/main", response_class=HTMLResponse)
async def main_page(
    request: Request,
    current_user: Optional[models.User] = Depends(get_current_user_optional),
):
    """메인 페이지 (관리자)."""
    if not current_user or current_user.role != "admin":
        return RedirectResponse(url="/", status_code=status.HTTP_303_SEE_OTHER)

    return templates.TemplateResponse(
        "main.html",
        {
            "request": request,
            "is_admin": True,
            "username": current_user.name or current_user.username,
            "title": "관리자 메인",
        },
    )


@app.get("/main_user", response_class=HTMLResponse)
async def main_user_page(
    request: Request,
    current_user: Optional[models.User] = Depends(get_current_user_optional),
):
    """메인 페이지 (일반 사용자)."""
    if not current_user or current_user.role == "admin":
        return RedirectResponse(url="/", status_code=status.HTTP_303_SEE_OTHER)

    return templates.TemplateResponse(
        "main.html",
        {
            "request": request,
            "is_admin": False,
            "username": current_user.name or current_user.username,
            "title": "일반 사용자 메인",
        },
    )


@app.get("/users", response_class=HTMLResponse)
async def users_page(
    request: Request,
    db: Session = Depends(get_db),
    current_user: Optional[models.User] = Depends(get_current_user_optional),
):
    """사용자 관리 페이지 (관리자 전용)."""
    if not current_user or current_user.role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="접근 권한이 없습니다.",
        )

    error_message = request.query_params.get("error")
    success_message = request.query_params.get("success")

    users = crud.get_users(db)

    return templates.TemplateResponse(
        "users.html",
        {
            "request": request,
            "users": users,
            "title": "사용자 관리",
            "error_message": error_message,
            "success_message": success_message,
        },
    )


@app.get("/logout")
async def logout():
    """로그아웃: 쿠키 삭제 후 루트로 이동."""
    response = RedirectResponse(url="/", status_code=status.HTTP_303_SEE_OTHER)
    response.delete_cookie(key="user_id")
    return response


# ============================================================
# 5. 로그인/회원 관련 API (웹 + 앱)
# ============================================================

# 5-1. 웹 로그인 API
@app.post("/login")
async def login_post(
    employee_number: str = Form(...),
    username: str = Form(...),
    password: str = Form(...),
    db: Session = Depends(get_db),
):
    print(f"[DEBUG] 로그인 시도: 사번={employee_number}, 이름={username}")

    user = (
        db.query(models.User)
        .filter(
            models.User.employee_number == employee_number,
            models.User.username == username,
        )
        .first()
    )

    if not user:
        print(
            f"[DEBUG] 로그인 실패: 사번({employee_number}) 또는 이름({username}) 불일치."
        )
        return RedirectResponse(
            url="/?error=Invalid credentials",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    if user.status == "deleted":
        print(f"[DEBUG] 로그인 실패: 사용자 ID={user.id}는 비활성화된 계정.")
        return RedirectResponse(
            url="/?error=Account disabled",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    try:
        valid = crud.verify_password(password, user.hashed_password)
    except Exception as e:
        print(f"[DEBUG] 비밀번호 검증 오류: {e}")
        return RedirectResponse(
            url="/?error=Invalid credentials",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    if not valid:
        print("[DEBUG] 로그인 실패: 비밀번호 불일치")
        return RedirectResponse(
            url="/?error=Invalid credentials",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    print(f"[DEBUG] 로그인 성공: 사용자 역할={user.role}")
    response = RedirectResponse(
        url="/main" if user.role == "admin" else "/main_user",
        status_code=status.HTTP_303_SEE_OTHER,
    )
    response.set_cookie(key="user_id", value=str(user.id), httponly=True)
    return response


# 5-2. 앱 전용 로그인 API
@app.post("/api/login", response_model=schemas.LoginResponse)
async def api_login(request: schemas.LoginRequest, db: Session = Depends(get_db)):
    """
    앱 전용 로그인 API
    - 입력: JSON (employee_number, username, password)
    - 출력: JSON (status, role, message, username)
    """
    print(f"[API LOGIN] 사번={request.employee_number}, 사용자명={request.username}")

    user = crud.get_user_by_employee(db, request.employee_number)
    if not user:
        print(f"[API LOGIN] 사번({request.employee_number}) 없음")
        return JSONResponse(
            content={"status": "error", "message": "Invalid employee number or password."},
            status_code=401,
        )

    if user.username != request.username:
        print(
            f"[API LOGIN] 사용자명 불일치 ({user.username} != {request.username})"
        )
        return JSONResponse(
            content={"status": "error", "message": "Invalid username."},
            status_code=401,
        )

    if user.status == "deleted":
        return JSONResponse(
            content={"status": "error", "message": "Account disabled."},
            status_code=403,
        )

    try:
        valid = crud.verify_password(request.password, user.hashed_password)
    except Exception as e:
        print(f"[API LOGIN] 비밀번호 검증 오류: {e}")
        return JSONResponse(
            content={"status": "error", "message": "Invalid password."},
            status_code=401,
        )

    if not valid:
        print("[API LOGIN] 비밀번호 불일치")
        return JSONResponse(
            content={"status": "error", "message": "Invalid credentials."},
            status_code=401,
        )

    print(f"[API LOGIN] 로그인 성공 - 역할: {user.role}")
    return JSONResponse(
        content={
            "status": "success",
            "role": user.role,
            "message": "Login successful.",
            "username": user.name,
        },
        status_code=200,
    )


# 5-3. 사용자 등록
@app.post("/register")
def register_post(
    employee_number: str = Form(...),
    username: str = Form(...),
    name: str = Form(...),
    password: str = Form(...),
    role: str = Form(...),
    db: Session = Depends(get_db),
):
    user_data = schemas.UserCreate(
        employee_number=employee_number,
        username=username,
        name=name,
        password=password,
        role=role,
    )

    if crud.get_user_by_username(db, user_data.username):
        error_msg = "이미 있는 계정입니다. 사원번호, 이름을 확인해주세요"
        return RedirectResponse(
            url=f"/users?error={error_msg}",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    try:
        crud.create_user(db, user_data)
        success_msg = f"사용자 '{name}'({username})가 성공적으로 등록되었습니다."
        return RedirectResponse(
            url=f"/users?success={success_msg}",
            status_code=status.HTTP_303_SEE_OTHER,
        )
    except Exception as e:
        print(f"[ERROR] User registration failed: {e}")
        return RedirectResponse(
            url="/users?error=사용자 등록 중 알 수 없는 오류가 발생했습니다.",
            status_code=status.HTTP_303_SEE_OTHER,
        )


# 5-4. 사용자 Soft Delete
@app.post("/users/{user_id}/soft_delete")
def soft_delete_user_status_post(user_id: int, db: Session = Depends(get_db)):
    """사용자의 상태를 'deleted'로 변경."""
    update_schema = schemas.UserUpdate(status="deleted")
    updated_user = crud.update_user(db, user_id, update_schema)

    if not updated_user:
        raise HTTPException(status_code=404, detail="사용자를 찾을 수 없습니다.")

    success_msg = f"사용자 ID {user_id} 계정이 비활성화되었습니다."
    return RedirectResponse(
        url=f"/users?success={success_msg}",
        status_code=status.HTTP_303_SEE_OTHER,
    )


# 5-5. 사용자 정보 업데이트
@app.post("/users/{user_id}/update")
def update_user_post(
    user_id: int,
    employee_number: str = Form(None),
    username: str = Form(None),
    name: str = Form(None),
    password: str = Form(None),
    role: str = Form(None),
    db: Session = Depends(get_db),
):
    update_data = {}
    if employee_number:
        update_data["employee_number"] = employee_number
    if username:
        update_data["username"] = username
    if name:
        update_data["name"] = name
    if password:
        update_data["password"] = password
    if role:
        update_data["role"] = role

    update_schema = schemas.UserUpdate(**update_data)
    updated_user = crud.update_user(db, user_id, update_schema)

    if not updated_user:
        error_msg = "사용자 정보 업데이트에 실패했습니다."
        return RedirectResponse(
            url=f"/users?error={error_msg}",
            status_code=status.HTTP_303_SEE_OTHER,
        )

    success_msg = f"사용자 '{updated_user.name}'의 정보가 성공적으로 업데이트되었습니다."
    return RedirectResponse(
        url=f"/users?success={success_msg}",
        status_code=status.HTTP_303_SEE_OTHER,
    )


# ============================================================
# 6. ROSBridge 연결 및 WebSocket(/ws/control)
# ============================================================

# ROSBridge 설정
ROSBRIDGE_IP = "192.168.0.100"
ROSBRIDGE_PORT = 9090

ros = roslibpy.Ros(host=ROSBRIDGE_IP, port=ROSBRIDGE_PORT)
ros_connected = False

# ROS 관련 상태 변수
main_loop = asyncio.get_event_loop()
latest_state = {"text": "대기 중"}
latest_amcl = None
latest_map = None
latest_batt = None
prev_amcl_pos = None
total_distance = 0.0
start_time = None
clients: List[WebSocket] = []


def amcl_callback(msg):
    """AMCL 위치 콜백: 이동 거리 누적."""
    global latest_amcl, prev_amcl_pos, total_distance
    latest_amcl = msg

    pos = msg["pose"]["pose"]["position"]
    x, y = pos["x"], pos["y"]

    if prev_amcl_pos is not None:
        dx = x - prev_amcl_pos["x"]
        dy = y - prev_amcl_pos["y"]
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist > 0.001:
            total_distance += dist

    prev_amcl_pos = {"x": x, "y": y}


def map_callback(msg):
    """맵 OccupancyGrid 콜백."""
    global latest_map
    latest_map = msg
    info = msg["info"]
    print(f"[ROS] /map 수신: {info['width']} x {info['height']}, res={info['resolution']}")


def batt_callback(msg):
    """배터리 상태 콜백."""
    global latest_batt
    latest_batt = msg


def cmdvel_callback(msg):
    """로봇 속도에 따른 상태 변경."""
    global latest_state

    lin = msg["linear"]["x"]
    ang = msg["angular"]["z"]

    if abs(lin) < 0.01 and abs(ang) < 0.01:
        new_state = "정지"
    elif abs(lin) > abs(ang):
        new_state = "전진중" if lin > 0 else "후진중"
    else:
        new_state = "회전중"

    if latest_state["text"] != new_state:
        latest_state["text"] = new_state


# ROS 토픽 정의
amcl_topic = roslibpy.Topic(ros, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped")
map_topic = roslibpy.Topic(ros, "/map", "nav_msgs/OccupancyGrid")
batt_topic = roslibpy.Topic(ros, "/battery_state", "sensor_msgs/msg/BatteryState")
cmdvel_pub = roslibpy.Topic(ros, "/cmd_vel", "geometry_msgs/Twist")
cmdvel_sub = roslibpy.Topic(ros, "/cmd_vel", "geometry_msgs/Twist")
patrol_pub = roslibpy.Topic(ros, "/patrol/cmd", "std_msgs/String")


def setup_ros_topics():
    amcl_topic.subscribe(amcl_callback)
    map_topic.subscribe(map_callback)
    batt_topic.subscribe(batt_callback)
    cmdvel_sub.subscribe(cmdvel_callback)
    print("[ROS] 토픽 구독 활성화")


async def try_connect_ros():
    """ROSBridge에 주기적으로 재연결 시도."""
    global ros_connected
    while True:
        try:
            if not ros.is_connected:
                print("[ROS] ROSBridge 연결 재시도 중...")
                ros.run()

                for _ in range(5):
                    if ros.is_connected:
                        break
                    await asyncio.sleep(1)

                if ros.is_connected:
                    ros_connected = True
                    setup_ros_topics()
                    print("[ROS] ROSBridge 연결 성공")
                else:
                    print("[ROS] ROSBridge 연결 실패, 재시도 예정")
            await asyncio.sleep(5)
        except Exception as e:
            print(f"[ROS] 연결 중 오류: {e}")
            await asyncio.sleep(5)


@app.on_event("startup")
async def startup_ros():
    asyncio.create_task(try_connect_ros())


async def broadcast(data: dict):
    """연결된 모든 WebSocket 클라이언트에게 브로드캐스트."""
    dead = []
    for ws in clients:
        try:
            await ws.send_json(data)
        except Exception:
            dead.append(ws)
    for d in dead:
        if d in clients:
            clients.remove(d)


@app.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    """
    로봇 제어 및 상태 전송용 WebSocket 엔드포인트.
    - 클라이언트 -> 서버: cmd_vel, patrol 명령
    - 서버 -> 클라이언트: map, amcl_pose, battery, distance, time, state
    """
    global total_distance, start_time
    await websocket.accept()
    clients.append(websocket)
    print(f"[WS] 클라이언트 연결됨 (총 {len(clients)}명)")

    try:
        while True:
            # 1) 클라이언트에서 들어오는 명령 처리
            try:
                msg = await asyncio.wait_for(websocket.receive_text(), timeout=0.01)
                data = json.loads(msg)
                t = data.get("type")

                if t == "patrol":
                    action = data.get("action")
                    if action == "single":
                        patrol_pub.publish(roslibpy.Message({"data": "start_once"}))
                        latest_state["text"] = "1회 순찰 중"
                        start_time = time.time()
                        total_distance = 0.0
                    elif action == "repeat":
                        patrol_pub.publish(roslibpy.Message({"data": "start_repeat"}))
                        latest_state["text"] = "반복 순찰 중"
                        start_time = time.time()
                        total_distance = 0.0
                    elif action == "return":
                        patrol_pub.publish(roslibpy.Message({"data": "return"}))
                        latest_state["text"] = "복귀 중"
                    elif action == "stop":
                        patrol_pub.publish(roslibpy.Message({"data": "stop"}))
                        latest_state["text"] = "정지"
                        cmdvel_pub.publish(
                            roslibpy.Message(
                                {
                                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                                }
                            )
                        )

                    await broadcast({"type": "state", "text": latest_state["text"]})
                    print(f"[WS] 순찰 명령 수신: {action}")

                elif t == "cmd_vel":
                    lin = float(data.get("linear", 0.0))
                    ang = float(data.get("angular", 0.0))
                    twist = {
                        "linear": {"x": lin, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": ang},
                    }
                    cmdvel_pub.publish(roslibpy.Message(twist))
            except asyncio.TimeoutError:
                pass

            # 2) ROS -> 클라이언트 주기적 데이터 송신
            await asyncio.sleep(0.2)

            # AMCL 위치, 누적 거리
            if latest_amcl:
                pos = latest_amcl["pose"]["pose"]["position"]
                ori = latest_amcl["pose"]["pose"]["orientation"]
                siny_cosp = 2 * (
                    ori["w"] * ori["z"] + ori["x"] * ori["y"]
                )
                cosy_cosp = 1 - 2 * (ori["y"] ** 2 + ori["z"] ** 2)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                await websocket.send_json(
                    {
                        "type": "amcl_pose",
                        "x": pos["x"],
                        "y": pos["y"],
                        "yaw": yaw,
                    }
                )
                await websocket.send_json(
                    {
                        "type": "distance",
                        "meters": round(total_distance, 2),
                    }
                )

            # 순찰 시간
            if start_time:
                elapsed = round((time.time() - start_time) / 60, 2)
                await websocket.send_json(
                    {
                        "type": "time",
                        "minutes": elapsed,
                    }
                )

            # 배터리
            if latest_batt:
                p = latest_batt.get("percentage", 0)
                if p <= 1:
                    p *= 100
                await websocket.send_json(
                    {
                        "type": "battery",
                        "percentage": int(round(p, 1)),
                    }
                )

            # 지도
            if latest_map:
                info = latest_map["info"]
                data = latest_map["data"]
                width, height = info["width"], info["height"]
                res = info["resolution"]
                origin = info["origin"]["position"]

                arr = np.array(data, dtype=np.int8).reshape(height, width)
                arr = np.flipud(arr)
                gray = np.zeros_like(arr, dtype=np.uint8)
                gray[arr == -1] = 205
                gray[arr == 0] = 255
                gray[arr > 0] = 0

                await websocket.send_json(
                    {
                        "type": "map",
                        "width": width,
                        "height": height,
                        "res": res,
                        "origin": {"x": origin["x"], "y": origin["y"]},
                        "gray": gray.flatten().tolist(),
                    }
                )

            # 현재 상태
            await websocket.send_json(
                {
                    "type": "state",
                    "text": latest_state["text"],
                }
            )

    except WebSocketDisconnect:
        print("[WS] 클라이언트 연결 종료")
    finally:
        if websocket in clients:
            clients.remove(websocket)


@app.on_event("shutdown")
def shutdown_event():
    try:
        amcl_topic.unsubscribe()
        batt_topic.unsubscribe()
        map_topic.unsubscribe()
        cmdvel_sub.unsubscribe()
        cmdvel_pub.unadvertise()
        patrol_pub.unadvertise()
        ros.terminate()
        print("[ROS] ROSBridge 연결 종료")
    except Exception:
        pass


# ============================================================
# 7. 실행부
# ============================================================

if __name__ == "__main__":
    # 로컬 테스트: host=0.0.0.0, port=8000
    uvicorn.run("main:app", host="0.0.0.0", port=8000)
