// ===========================
// 1. 연결 설정
// ===========================
const WS_HOST = "52.78.32.242"; // EC2 IP
//const WS_HOST = "192.168.0.57"; // local IP
const WS_PORT = 8000;
//const WS_URL = `wss://${WS_HOST}:${WS_PORT}/ws/control`;
const WS_URL = `wss://${WS_HOST}/ws/control`;
const CAM_HOST = "192.168.0.100"; // Raspberry Pi (web_video_server)
const MJPEG_STREAM_URL = `http://${CAM_HOST}:8080/stream?topic=/image_raw&type=ros_compressed&width=640&height=480&quality=50`;

// ===========================
// 2. DOM 요소 캐시
// ===========================
const statusIndicator = document.getElementById("status-indicator");
const statusText = document.getElementById("status-text");
const robotStatusEl = document.getElementById("metric-status");

const mapMain = document.getElementById("map-main");
const videoMain = document.getElementById("video-main");
const mapSmall = document.getElementById("map-small");
const videoSmall = document.getElementById("video-small");

const webcamFeed = document.getElementById("webcam-feed");
const webcamSmallFeed = document.getElementById("webcam-small-feed");

const patrolControlMap = document.getElementById("patrol-control-map");
const robotLocationCard = document.getElementById("robot-location-card");
const manualControl = document.getElementById("manual-control");

const elBatt = document.getElementById("metric-battery");
const elDist = document.getElementById("metric-distance");
const elTime = document.getElementById("metric-time");
const robotLocation = document.getElementById("robot-location");

const mapCanvas = document.getElementById("map-canvas");
const mapCtx = mapCanvas.getContext("2d", { willReadFrequently: true });
const mapContainer = mapCanvas.closest(".w-full.h-full");

// ===========================
// 3. 상태 정의
// ===========================
const statusConfigs = {
    순찰중: { class: "bg-green-200 text-green-800", icon: "fas fa-route" },
    "1회 순찰 중": { class: "bg-green-200 text-green-800", icon: "fas fa-route" },
    "반복 순찰 중": { class: "bg-green-200 text-green-800", icon: "fas fa-sync-alt" },
    복귀중: { class: "bg-indigo-200 text-indigo-800", icon: "fas fa-home" },
    "복귀 중": { class: "bg-indigo-200 text-indigo-800", icon: "fas fa-home" },
    충전중: {
        class: "bg-blue-200 text-blue-800",
        icon: "fas fa-charging-station",
    },
    정지: { class: "bg-red-200 text-red-800", icon: "fas fa-stop-circle" },
    전진중: { class: "bg-blue-200 text-blue-800", icon: "fas fa-arrow-up" },
    후진중: { class: "bg-blue-200 text-blue-800", icon: "fas fa-arrow-down" },
    회전중: { class: "bg-blue-200 text-blue-800", icon: "fas fa-sync-alt" },
    임무완료: {
        class: "bg-teal-200 text-teal-800",
        icon: "fas fa-check-circle",
    },
    "대기 중": {
        class: "bg-gray-200 text-gray-800",
        icon: "fas fa-ellipsis-h",
    },
    "연결 오류": {
        class: "bg-red-200 text-red-800",
        icon: "fas fa-exclamation-triangle",
    },
    "연결 끊김": {
        class: "bg-yellow-200 text-yellow-800",
        icon: "fas fa-plug",
    },
};

function updateRobotStatus(key, currentZone = "", task = "", zoneLabel = "") {
    let cfg = statusConfigs[key];

    if (!cfg) {
        if (key.includes("순찰")) {
            // 구역 순찰이든, 전체 순찰이든 다 같은 스타일 사용
            cfg = statusConfigs["순찰중"] || statusConfigs["1회 순찰 중"];
        } else if (key.includes("복귀")) {
            cfg = statusConfigs["복귀 중"];
        } else if (key.includes("정지") || key.includes("일시정지")) {
            cfg = statusConfigs["정지"];
        } else {
            cfg = statusConfigs["대기 중"];
        }
    }

    // 2) 어떤 구역 텍스트를 보여줄지 결정
    //   - currentZone : PatrolManager 에서 보내주는 현재 구역 (예: "A")
    //   - zoneLabel   : 프론트에서 만든 라벨 (예: "A", "A → B → C")
    let zoneName = "";
    if (currentZone && currentZone.length > 0) {
        zoneName = currentZone;              // 실제 현재 구역이 있으면 그걸 우선
    } else if (zoneLabel && zoneLabel.length > 0) {
        zoneName = zoneLabel;               // 아직 status가 안 들어왔을 때 임시 표시
    }

    let extra = "";
    if (zoneName) {
        if (task === "zone_seq" || zoneName.includes("→")) {
            // 시퀀스 순찰일 때는 “선택 구역 시퀀스” 형식으로
            extra = `
              <div class="mt-1 text-sm text-gray-600">
                선택 구역 시퀀스:
                <span class="font-bold">${zoneName}</span>
              </div>`;
        } else {
            // 단일 구역 순찰 또는 “현재 구역” 정보 있을 때
            extra = `
              <div class="mt-1 text-sm text-gray-600">
                (현재 <span class="font-bold">${zoneName}</span> 구역)
              </div>`;
        }
    }

    // 3) 뱃지 렌더링
    robotStatusEl.innerHTML = `
      <div class="flex flex-col items-center">
        <div class="flex items-center">
          <i class="${cfg.icon} mr-3"></i> ${key}
        </div>
        ${extra}
      </div>
    `;
    robotStatusEl.className = "status-badge " + cfg.class;
}

// ===========================
// 4. WebSocket 통신
// ===========================
let ws = null,
    isConnected = false;
let mapBuffer = null,
    mapBufferCtx = null;
let mapMeta = { w: 0, h: 0, res: 0.05, ox: 0, oy: 0 };
let robotPose = { x: null, y: null, yaw: 0 };
let zonesOverlay = {};
let selectedZones = [];
let selectedMode = "once";

function connectWS() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        isConnected = true;
        statusIndicator.classList.replace("bg-red-500", "bg-green-500");
        statusText.textContent = "서버 연결됨";
        statusText.classList.replace("text-red-700", "text-green-700");
        updateRobotStatus("대기 중");
    };

    ws.onerror = () => {
        statusIndicator.classList.replace("bg-green-500", "bg-red-500");
        statusText.textContent = "서버 오류";
        statusText.classList.replace("text-green-700", "text-red-700");
        updateRobotStatus("연결 오류");
    };

    ws.onclose = () => {
        isConnected = false;
        statusIndicator.classList.replace("bg-green-500", "bg-yellow-500");
        statusText.textContent = "서버 연결 끊김";
        statusText.classList.replace("text-green-700", "text-yellow-700");
        updateRobotStatus("연결 끊김");
        setTimeout(connectWS, 2000);
    };

    ws.onmessage = (event) => {
        let data = null;
        try {
            data = JSON.parse(event.data);
        } catch (e) {
            return;
        }
        handleMessage(data);
    };
}

function wsSend(obj) {
    if (isConnected) ws.send(JSON.stringify(obj));
}

// ===========================
// 5. 메시지 처리
// ===========================
function handleMessage(m) {
    switch (m.type) {
        case "map":
            drawMap(m);
            break;
        case "amcl_pose":
            updatePose(m);
            break;
        case "battery":
            elBatt.textContent = `${m.percentage ?? 0}%`;
            break;
        case "distance":
            elDist.textContent = `${(m.meters ?? 0).toFixed(2)} m`;
            break;
        case "time":
            elTime.textContent = `${(m.minutes ?? 0).toFixed(1)} min`;
            break;
        case "state":
            updateRobotStatus(m.text, m.current_zone || "", m.task || "");
            break;
        case "env":
            if (m.temp !== null)
                document.getElementById(
                    "metric-temp"
                ).textContent = `${m.temp.toFixed(1)} °C`;
            if (m.hum !== null)
                document.getElementById(
                    "metric-humi"
                ).textContent = `${m.hum.toFixed(1)} %`;
            if (m.co2 !== null)
                document.getElementById(
                    "metric-co2"
                ).textContent = `${m.co2.toFixed(0)} ppm`;
            if (m.tvoc !== null)
                document.getElementById(
                    "metric-tvoc"
                ).textContent = `${m.tvoc.toFixed(0)} ppb`;
            if (m.lpg !== null)
                document.getElementById(
                    "metric-lpg"
                ).textContent = `${m.lpg.toFixed(2)} ppm`;
            if (m.co !== null)
                document.getElementById(
                    "metric-co"
                ).textContent = `${m.co.toFixed(0)} ppm`;
            break;
        case "fire_alarm": // <<<<<<<<<<<<<<< 화재 감지 알람 처리 추가
            showFireAlert(m.message, m.lpg);
            break;
        case "zones":
            zonesOverlay = m.zones || {};
            drawRobot();
            break;
    }
}

// ===========================
// 5.1. 서버 상태 초기화 전송 함수
// ===========================
function sendAlertClearSignal() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(
            JSON.stringify({
                type: "alert_clear", // 서버가 인식할 메시지 타입
            })
        );
        console.log("서버에 화재 알림 상태 초기화 요청 전송.");
    }
}

function updatePose(m) {
    robotPose = { x: m.x, y: m.y, yaw: m.yaw };

    const xf = m.x?.toFixed(2) ?? "0.00";
    const yf = m.y?.toFixed(2) ?? "0.00";

    robotLocation.textContent = `X=${xf} m, Y=${yf} m`;

    drawRobot();
}

// ===========================
// 6. 지도 및 로봇 표시
// ===========================
// === 지도 렌더링 + 로봇 표시 (회전 + 비율 유지) ===
function drawMap(m) {
    const w = m.width,
        h = m.height,
        gray = m.gray;
    if (!Array.isArray(gray) || gray.length !== w * h) return;

    // 오프스크린 버퍼 준비
    if (!mapBuffer) {
        mapBuffer = document.createElement("canvas");
        mapBufferCtx = mapBuffer.getContext("2d");
    }
    if (mapBuffer.width !== w || mapBuffer.height !== h) {
        mapBuffer.width = w;
        mapBuffer.height = h;
    }

    // occupancy → grayscale 이미지로 변환
    const imgData = new ImageData(w, h);
    for (let i = 0, j = 0; i < gray.length; i++, j += 4) {
        const g = gray[i] | 0;
        imgData.data[j] = g;
        imgData.data[j + 1] = g;
        imgData.data[j + 2] = g;
        imgData.data[j + 3] = 255;
    }
    mapBufferCtx.putImageData(imgData, 0, 0);

    // 메타데이터 갱신
    mapMeta = {
        w,
        h,
        res: m.res ?? mapMeta.res,
        ox: m.origin?.x ?? 0,
        oy: m.origin?.y ?? 0,
    };

    // 지도 + 로봇 다시 그리기
    drawRobot();
}

// world → pixel (서버에서 이미 flipud 했으므로 y축 재반전만)
function worldToPixel(x, y) {
    // world → pixel 좌표 (y 반전)
    const px = (x - mapMeta.ox) / mapMeta.res;
    const py = mapMeta.h - (y - mapMeta.oy) / mapMeta.res;
    return { x: px, y: py };
}

function drawRobot() {
    if (!mapBuffer || !mapMeta.w || !mapMeta.h) return;

    const viewW = mapCanvas.clientWidth || mapCanvas.width || 1;
    const viewH = mapCanvas.clientHeight || mapCanvas.height || 1;

    // 캔버스 크기 동기화
    if (mapCanvas.width !== viewW || mapCanvas.height !== viewH) {
        mapCanvas.width = viewW;
        mapCanvas.height = viewH;
    }

    // === 1) 비율 유지 스케일 + 중앙 정렬 offset 계산 ===
    const scale = Math.min(viewW / mapMeta.w, viewH / mapMeta.h);
    const offX = (viewW - mapMeta.w * scale) / 2;
    const offY = (viewH - mapMeta.h * scale) / 2;

    // === 2) 초기화 후 변환 설정 (※ 회전/translate 절대 안 건드림) ===
    mapCtx.setTransform(1, 0, 0, 1, 0, 0);
    mapCtx.clearRect(0, 0, viewW, viewH);

    // 지도 그리기
    mapCtx.setTransform(scale, 0, 0, scale, offX, offY);
    mapCtx.drawImage(mapBuffer, 0, 0);

    // === 3) 로봇 그리기 ===
    if (robotPose.x != null && robotPose.y != null) {
        const p = worldToPixel(robotPose.x, robotPose.y);
        const size = 3;

        mapCtx.save();
        mapCtx.translate(p.x, p.y);
        mapCtx.rotate(robotPose.yaw); // 로봇 방향은 여기서만 회전

        // 로봇 위치 점
        mapCtx.fillStyle = "#ef4444";
        mapCtx.beginPath();
        mapCtx.arc(0, 0, size, 0, 2 * Math.PI);
        mapCtx.fill();

        mapCtx.restore();
    }

    if (zonesOverlay && Object.keys(zonesOverlay).length > 0) {
        mapCtx.lineWidth = 1;  // 테두리 얇게
        mapCtx.strokeStyle = "#6366f1";

        for (const [zname, poly] of Object.entries(zonesOverlay)) {
            if (!Array.isArray(poly) || poly.length < 3) continue;

            // === 1) 테두리 그리기 ===
            mapCtx.beginPath();
            poly.forEach(([wx, wy], idx) => {
                const p = worldToPixel(wx, wy);
                if (idx === 0) mapCtx.moveTo(p.x, p.y);
                else mapCtx.lineTo(p.x, p.y);
            });
            mapCtx.closePath();
            mapCtx.stroke();

            // === 2) 중앙값(centroid) 계산 ===
            let cx = 0, cy = 0;
            poly.forEach(([wx, wy]) => {
                cx += wx;
                cy += wy;
            });
            cx /= poly.length;
            cy /= poly.length;

            const center = worldToPixel(cx, cy);

            // === 3) 중앙에 텍스트 표시 ===
            mapCtx.fillStyle = "#111827";
            mapCtx.font = "bold 15px sans-serif"; // 글씨 좀 더 크게
            mapCtx.textAlign = "center";
            mapCtx.textBaseline = "middle";
            mapCtx.fillText(zname, center.x, center.y);
        }
    }


    // === 4) 순찰 구역 폴리곤 (지도 좌표계 그대로 사용) ===
    if (patrolAreaPoints.length > 0) {
        mapCtx.beginPath();
        mapCtx.strokeStyle = "#ef4444";
        mapCtx.lineWidth = 4;
        mapCtx.fillStyle = "rgba(239,68,68,0.2)";
        mapCtx.moveTo(patrolAreaPoints[0].x, patrolAreaPoints[0].y);
        for (let i = 1; i < patrolAreaPoints.length; i++) {
            mapCtx.lineTo(patrolAreaPoints[i].x, patrolAreaPoints[i].y);
        }
        mapCtx.closePath();
        mapCtx.stroke();
        mapCtx.fill();

        patrolAreaPoints.forEach((p) => {
            mapCtx.beginPath();
            mapCtx.fillStyle = "#10b981";
            mapCtx.arc(p.x, p.y, 6, 0, 2 * Math.PI);
            mapCtx.fill();
        });
    }
}

// ===========================
// 7. 웹캠 & 뷰 전환
// ===========================
let isVideoRunning = false;
let currentMainView = "map";

window.toggleMainView = function (view) {
    if (view === currentMainView) return;
    if (view === "map") {
        mapMain.classList.remove("hidden");
        videoMain.classList.add("hidden");
        mapSmall.classList.add("hidden");
        videoSmall.classList.remove("hidden");
        patrolControlMap.classList.remove("hidden");
        robotLocationCard.classList.remove("hidden");
        manualControl.classList.add("hidden");
        currentMainView = "map";
    } else {
        videoMain.classList.remove("hidden");
        mapMain.classList.add("hidden");
        videoSmall.classList.add("hidden");
        mapSmall.classList.remove("hidden");
        manualControl.classList.remove("hidden");
        patrolControlMap.classList.add("hidden");
        robotLocationCard.classList.add("hidden");
        currentMainView = "video";
        if (!isVideoRunning) toggleVideoFeed(true);

        wsSend({ type: "patrol", action: "stop" });
    }
};

window.toggleVideoFeed = function (forceStart) {
    const newState =
        typeof forceStart === "boolean" ? forceStart : !isVideoRunning;
    const button = document.querySelector(
        '.btn-action[onclick="toggleVideoFeed()"]'
    );

    if (newState) {
        webcamFeed.src = MJPEG_STREAM_URL;
        webcamSmallFeed.src = MJPEG_STREAM_URL;
        document.getElementById("video-placeholder")?.classList.add("hidden");
        if (button) {
            button.innerHTML =
                '<i class="fas fa-camera-slash mr-2"></i> 웹캠 중지';
            button.classList.replace("bg-blue-500", "bg-red-500");
        }
        isVideoRunning = true;
    } else {
        webcamFeed.src = "";
        webcamSmallFeed.src = "";
        document
            .getElementById("video-placeholder")
            ?.classList.remove("hidden");
        if (button) {
            button.innerHTML = '<i class="fas fa-camera mr-2"></i> 웹캠 시작';
            button.classList.replace("bg-red-500", "bg-blue-500");
        }
        isVideoRunning = false;
    }
};

// ===========================
// X. 구역 선택 관련 함수
// ===========================
window.toggleZoneSelect = function (zone) {
    const idx = selectedZones.indexOf(zone);
    if (idx >= 0) {
        selectedZones.splice(idx, 1);
    } else {
        selectedZones.push(zone);
    }
    updateZoneButtonsUI();
};

window.setZoneMode = function (mode) {
    // mode: "once" | "repeat"
    selectedMode = mode;
    updateZoneModeUI();
};

window.runZonePatrol = function () {
    if (!isConnected) {
        customAlert("서버와 연결되어 있지 않습니다.");
        return;
    }
    if (selectedZones.length === 0) {
        customAlert("최소 1개 이상의 구역을 선택하세요.");
        return;
    }

    wsSend({
        type: "zone_patrol",
        mode: selectedMode,     // "once" or "repeat"
        zones: selectedZones,   // ["A"], ["A","C","D"] 등
    });

    const label = selectedZones.join(" → ");
    const modeText = selectedMode === "repeat" ? "반복" : "1회";

    updateRobotStatus(
        `${modeText} 구역 순찰`,
        "",
        selectedZones.length > 1 ? "zone_seq" : "zone_route",
        label
    );

    customAlert(`${label} 구역(${modeText}) 순찰을 시작합니다.`);
};

// UI 반영용
function updateZoneButtonsUI() {
    ["A","B","C","D"].forEach((z) => {
        const btn = document.getElementById(`zone-btn-${z}`);
        if (!btn) return;
        if (selectedZones.includes(z)) {
            btn.classList.add("bg-indigo-600", "text-white");
            btn.classList.remove("bg-gray-100", "text-gray-700");
        } else {
            btn.classList.remove("bg-indigo-600", "text-white");
            btn.classList.add("bg-gray-100", "text-gray-700");
        }
    });
}

function updateZoneModeUI() {
    ["once", "repeat"].forEach((m) => {
        const btn = document.getElementById(`zone-mode-${m}`);
        if (!btn) return;
        if (selectedMode === m) {
            btn.classList.add("bg-blue-600", "text-white");
            btn.classList.remove("bg-gray-100", "text-gray-700");
        } else {
            btn.classList.remove("bg-blue-600", "text-white");
            btn.classList.add("bg-gray-100", "text-gray-700");
        }
    });
}

// ===========================
// 8. 명령 전송 (미션, 수동 제어)
// ===========================
window.publishCommand = function (command) {
    if (!isConnected) return;

    let action = null;
    let status = null;

    switch (command) {
        case "forward":
            action = "manual_forward";
            status = "수동 전진";
            break;
        case "backward":
            action = "manual_backward";
            status = "수동 후진";
            break;
        case "left":
            action = "manual_left";
            status = "수동 좌회전";
            break;
        case "right":
            action = "manual_right";
            status = "수동 우회전";
            break;
        case "stop":
            action = "manual_stop";
            status = "정지";
            break;
        default:
            return;
    }

    wsSend({ type: "patrol", action });   // ← /patrol/cmd 로 문자열 전송
    if (status) updateRobotStatus(status);
};

window.publishMission = function (missionType) {
    if (!isConnected) return;
    const map = {
        single: {
            text: "1회 순찰을 시작합니다.",
            action: "single",
            status: "순찰중",
        },
        repeat: {
            text: "반복 순찰을 시작합니다.",
            action: "repeat",
            status: "순찰중",
        },
        return: {
            text: "복귀 명령을 전송했습니다.",
            action: "return",
            status: "복귀중",
        },
        pause: {
            text: "순찰을 일시정지했습니다.",
            action: "pause",
            status: "정지",
        },
        resume: {
            text: "순찰을 재개합니다.",
            action: "resume",
            status: "순찰중",
        },
    };
    const m = map[missionType];
    if (!m) return;
    wsSend({ type: "patrol", action: m.action });
    updateRobotStatus(m.status);
    customAlert(m.text);
};

window.publishPatrolStop = function () {
    if (!isConnected) return;

    wsSend({ type: "patrol", action: "stop" });

    updateRobotStatus("정지");
    customAlert("순찰을 즉지 정지했습니다.");
};

// ===========================
// 9. 순찰 구역 그리기 (폴리곤)
// ===========================
let isDrawingPatrolArea = false;
let patrolAreaPoints = [];

function handleMapClick(e) {
    if (!isDrawingPatrolArea) return;

    const rect = mapCanvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    // 현재 회전/스케일 고려한 실제 좌표 변환 (역변환)
    const viewW = mapCanvas.clientWidth;
    const viewH = mapCanvas.clientHeight;
    const scale = Math.min(viewW / mapMeta.w, viewH / mapMeta.h);
    const offX = (viewW - mapMeta.w * scale) / 2;
    const offY = (viewH - mapMeta.h * scale) / 2;

    const px = (x - offX) / scale;
    const py = (y - offY) / scale;

    patrolAreaPoints.push({ x: px, y: py });
    drawRobot();
}

function drawPatrolArea() {
    const ctx = mapCanvas.getContext("2d");
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    if (patrolAreaPoints.length > 0) {
        ctx.strokeStyle = "#ef4444";
        ctx.lineWidth = 4;
        ctx.fillStyle = "rgba(239,68,68,0.2)";
        ctx.beginPath();
        ctx.moveTo(patrolAreaPoints[0].x, patrolAreaPoints[0].y);
        for (let i = 1; i < patrolAreaPoints.length; i++)
            ctx.lineTo(patrolAreaPoints[i].x, patrolAreaPoints[i].y);
        ctx.closePath();
        ctx.stroke();
        ctx.fill();

        patrolAreaPoints.forEach((p) => {
            ctx.fillStyle = "#10b981";
            ctx.beginPath();
            ctx.arc(p.x, p.y, 6, 0, 2 * Math.PI);
            ctx.fill();
        });
    }
}

window.startDrawingPatrolArea = function () {
    if (isDrawingPatrolArea) {
        stopDrawingPatrolArea(true);
        return;
    }
    isDrawingPatrolArea = true;
    patrolAreaPoints = [];
    mapContainer.classList.add("drawing-active");
    const btn = document.getElementById("draw-patrol-btn");
    btn.classList.replace("bg-purple-500", "bg-red-500");
    btn.innerHTML = '<i class="fas fa-times-circle mr-2"></i> 구역 그리기 취소';
    mapCanvas.addEventListener("click", handleMapClick);
};

window.stopDrawingPatrolArea = function (reset = true) {
    isDrawingPatrolArea = false;
    mapContainer.classList.remove("drawing-active");
    mapCanvas.removeEventListener("click", handleMapClick);
    const btn = document.getElementById("draw-patrol-btn");
    btn.classList.replace("bg-red-500", "bg-purple-500");
    btn.innerHTML =
        '<i class="fas fa-mouse-pointer mr-2"></i> 지도에 구역 그리기 시작';
    document.getElementById("set-patrol-btn").classList.add("hidden");
    if (reset) {
        patrolAreaPoints = [];
        const ctx = mapCanvas.getContext("2d");
        ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    }
};

window.setPatrolArea = function () {
    if (patrolAreaPoints.length < 3) {
        customAlert("3개 이상의 점을 지정해야 합니다.");
        return;
    }
    stopDrawingPatrolArea(false);
    console.table(patrolAreaPoints);
    customAlert("순찰 구역 설정 완료!");
};

// ===========================
// 10. 커스텀 알림
// ===========================
window.customAlert = function (message) {
    const tempDiv = document.createElement("div");
    tempDiv.innerHTML = `
    <div class="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div class="bg-white p-6 rounded-xl shadow-2xl max-w-md w-full">
        <p class="text-xl font-bold text-blue-600 mb-4 flex items-center">
          <i class="fas fa-info-circle mr-2"></i> 시스템 알림
        </p>
        <p class="text-gray-700 mb-6">${message}</p>
        <button onclick="this.closest('.fixed').remove()" class="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded-lg w-full transition">확인</button>
      </div>
    </div>`;
    document.body.appendChild(tempDiv);
};

// 화재 감지 알람 전용 함수 추가
window.showFireAlert = function (message, lpg_value) {
    // 알람이 이미 떠있으면 중복 방지
    if (document.getElementById("fire-alert-modal")) return;

    const tempDiv = document.createElement("div");
    tempDiv.id = "fire-alert-modal";
    tempDiv.innerHTML = `
    <div class="fixed inset-0 bg-red-800 bg-opacity-75 flex items-center justify-center z-[100]">
      <div class="bg-white p-8 rounded-2xl shadow-fire max-w-lg w-full transform transition-all scale-100 border-4 border-red-500">
        <p class="text-3xl font-extrabold text-red-600 mb-4 flex items-center justify-center animate-pulse">
          <i class="fas fa-exclamation-triangle mr-3"></i> 긴급 화재 감지 알림
        </p>
        <p class="text-xl text-gray-800 mb-6 text-center font-semibold">
          ${message}
        </p>
        <p class="text-lg text-red-500 mb-8 text-center">
          현재 LPG 수치: ${lpg_value.toFixed(2)} ppm
        </p>
        
        <div class="space-y-4">
          <button 
            onclick="publishPatrolStop(); this.closest('.fixed').remove(); customAlert('로봇 긴급 정지 명령 전송 완료.'); sendAlertClearSignal();" 
            class="w-full bg-red-600 hover:bg-red-700 text-white font-bold py-3 px-4 rounded-lg transition duration-200 shadow-md flex items-center justify-center">
            <i class="fas fa-stop-circle mr-2"></i> 로봇 긴급 정지
          </button>
          
          <button
            onclick="publishMission('return'); this.closest('.fixed').remove(); customAlert('로봇 복귀 명령 전송 완료.'); sendAlertClearSignal();" 
            class="w-full bg-yellow-500 hover:bg-yellow-600 text-gray-900 font-bold py-3 px-4 rounded-lg transition duration-200 shadow-md flex items-center justify-center">
            <i class="fas fa-home mr-2"></i> 즉시 복귀
          </button>

          <button 
            onclick="this.closest('.fixed').remove(); sendAlertClearSignal();"
            class="w-full bg-gray-300 hover:bg-gray-400 text-gray-800 font-bold py-3 px-4 rounded-lg transition duration-200">
            알림 확인
          </button>
        </div>
      </div>
    </div>`;
    document.body.appendChild(tempDiv);
};

// ===========================
// 11. 초기화
// ===========================
window.onload = function () {
    connectWS();
    toggleMainView("map");
    toggleVideoFeed(false);
};
