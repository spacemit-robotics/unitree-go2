/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
(() => {
  "use strict";

  const applicationId = "unitree-go2-sdk";
  let applicationVerified = false;
  const conservativeLimits = [0.35, 0.25, 0.50];
  const modeLimits = {
    walk: [1.00, 0.40, 1.00],
    // CTS comprehensive-terrain physical envelope: half of the trained
    // translation range, full trained yaw range.
    terrain: [1.00, 0.50, 2.00],
  };
  const axes = [0, 0, 0];
  const heldKeys = new Set();
  const driveKeys = new Set(["KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE"]);
  // The standalone simple package is intended for a trusted local network;
  // unlike v9 it does not require a per-process access token.
  let token = "simple";
  let pendingMode = null;
  let selectedMode = "terrain";
  let sending = false;
  let cameraEnabled = false;
  let fpvEnabled = false;
  let fpvAvailable = false;
  let fpvPending = false;
  let fpvRemote = false;
  let fpvRevision = 0;
  let controlSource = "web";
  let selectedRobot = null;
  let robotOnline = false;
  let robotGeneration = 0;
  let switchingRobot = false;
  let robots = [];
  let listingRobots = false;
  let checkingStatus = null;
  let peer = null;
  let channel = null;
  let connecting = false;
  let lastAckAt = 0;
  let lastAckSequence = -1;
  let commandSequence = 0;
  let nextConnectAt = 0;

  const $ = (selector) => document.querySelector(selector);
  const connection = $("#connection");
  const tokenDialog = $("#token-dialog");
  const tokenInput = $("#token-input");
  const tokenError = $("#token-error");
  const lightToggle = $("#light-toggle");
  const fpvToggle = $("#fpv-toggle");
  const cameraToggle = $("#camera-toggle");
  const cameraStream = $("#camera-stream");
  const cameraPlaceholder = $("#camera-placeholder");
  const cameraStatus = $("#camera-status");
  const landscapeButton = $("#enter-landscape");
  const rotateMessage = $("#rotate-message");
  const robotsButton = $("#robots-button");
  const robotsDialog = $("#robots-dialog");
  const robotsList = $("#robots-list");

  async function request(path, options = {}, timeout = 1800) {
    if (options.method === "POST" && !applicationVerified) throw new Error("请连接 Go2 SDK 网页");
    const controller = new AbortController();
    const timer = window.setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(path, {
        ...options, headers: { ...options.headers, "X-Go2-Application": applicationId }, signal: controller.signal,
      });
      // Keep the deadline active while reading the body. A dropped mobile
      // connection can deliver headers and then leave the body pending.
      const payload = await response.json();
      const identity = path === "/api/robots" || /^\/api\/robots\/[^/]+\/status$/.test(path);
      if (response.ok && identity) {
        applicationVerified = payload.application === applicationId;
        if (!applicationVerified) {
          disconnectControl();
          throw new Error("当前地址不是 Go2 SDK 网页服务");
        }
      }
      return { ok: response.ok, payload };
    } finally {
      window.clearTimeout(timer);
    }
  }

  function api(path, robot = selectedRobot) {
    return `/api/robots/${robot}/${path}`;
  }

  function controlEnabled() {
    return applicationVerified && robotOnline && !switchingRobot && controlSource !== "sbus";
  }

  function streamAlive() {
    return channel?.readyState === "open" && performance.now() - lastAckAt < 1200;
  }

  function disconnectControl() {
    const previous = peer;
    peer = channel = null;
    connecting = false;
    lastAckAt = 0;
    lastAckSequence = -1;
    robotOnline = false;
    axes.fill(0);
    heldKeys.clear();
    pendingMode = null;
    showAxes();
    document.querySelectorAll(".joystick").forEach((stick) => stick.dispatchEvent(new Event("go2-stick-reset")));
    previous?.close();
    updateControls();
    setConnection(false, "正在重连…");
    nextConnectAt = performance.now() + 500;
  }

  async function connectControl() {
    if (!applicationVerified || !selectedRobot || switchingRobot || connecting || document.hidden || performance.now() < nextConnectAt) return;
    if (peer) {
      if (streamAlive() || (channel?.readyState !== "open" && performance.now() - lastAckAt < 5000)) return;
      disconnectControl();
      return;
    }
    connecting = true;
    const generation = robotGeneration;
    const target = selectedRobot;
    const current = new RTCPeerConnection({ iceServers: [] });
    peer = current;
    const data = current.createDataChannel("go2-velocity", { ordered: false, maxPacketLifeTime: 100 });
    channel = data;
    data.onopen = () => {
      if (peer !== current) return;
      lastAckAt = performance.now();
      sendCommand(true);
    };
    data.onmessage = (event) => {
      if (peer !== current || generation !== robotGeneration) return;
      let packet;
      try { packet = JSON.parse(event.data); } catch (_) { return; }
      if (!Number.isInteger(packet.seq) || packet.seq <= lastAckSequence || packet.seq > commandSequence) return;
      lastAckSequence = packet.seq;
      lastAckAt = performance.now();
      robotOnline = Boolean(packet.connected);
      const source = packet.control_source || "web";
      if (!robotOnline || source !== controlSource) {
        axes.fill(0);
        heldKeys.clear();
        pendingMode = null;
        showAxes();
      }
      controlSource = source;
      setConnection(robotOnline);
      updateControls();
    };
    data.onclose = data.onerror = () => { if (peer === current) disconnectControl(); };
    current.onconnectionstatechange = () => {
      if (peer === current && ["failed", "closed"].includes(current.connectionState)) disconnectControl();
    };
    try {
      await current.setLocalDescription(await current.createOffer());
      await new Promise((resolve, reject) => {
        if (current.iceGatheringState === "complete") return resolve();
        const timer = window.setTimeout(() => reject(new Error("连接建立超时")), 5000);
        const changed = () => {
          if (current.iceGatheringState !== "complete") return;
          window.clearTimeout(timer);
          current.removeEventListener("icegatheringstatechange", changed);
          resolve();
        };
        current.addEventListener("icegatheringstatechange", changed);
        changed();
      });
      if (peer !== current) return;
      const result = await request(api("rtc/offer", target), {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ type: "offer", sdp: current.localDescription.sdp }),
      }, 10000);
      if (peer !== current) return;
      if (!result.ok) throw new Error(result.payload.error || "连接建立失败");
      await current.setRemoteDescription(result.payload);
      lastAckAt = performance.now();
    } catch (_) {
      if (peer === current) disconnectControl();
    } finally {
      if (peer === current) connecting = false;
    }
  }

  function updateControls() {
    document.querySelectorAll(".action-button").forEach((button) => {
      button.disabled = !controlEnabled() || sending;
    });
    lightToggle.disabled = cameraToggle.disabled = !robotOnline || switchingRobot;
    fpvToggle.disabled = !selectedRobot || switchingRobot || fpvPending || fpvRemote || (!fpvAvailable && !fpvEnabled);
  }

  function renderRobots() {
    const currentIds = new Set(robots.map((robot) => robot.robot));
    robotsList.querySelectorAll("[data-robot]").forEach((button) => {
      if (!currentIds.has(button.dataset.robot)) button.remove();
    });
    for (const robot of robots) {
      let button = robotsList.querySelector(`[data-robot="${robot.robot}"]`);
      if (!button) {
        button = document.createElement("button");
        button.type = "button";
        button.dataset.robot = robot.robot;
        button.append(document.createElement("strong"), document.createElement("span"), document.createElement("small"));
        button.addEventListener("click", () => selectRobot(robot.robot));
        robotsList.append(button);
      }
      button.className = `robot-card${robot.robot === selectedRobot ? " selected" : ""}`;
      const name = button.querySelector("strong");
      name.textContent = robot.name;
      const status = button.querySelector("span");
      status.textContent = robot.connected ? "在线" : "未连接";
      status.className = robot.connected ? "robot-online" : "";
      const network = button.querySelector("small");
      network.textContent = robot.network_mode === "ap" ? `热点 · ${robot.ssid}`
        : robot.network_mode === "wifi" ? `Wi-Fi · ${robot.ssid}` : "当前网络不可达";
      button.disabled = !robot.connected || switchingRobot;
    }
    const selected = robots.find((robot) => robot.robot === selectedRobot);
    robotsButton.textContent = `${selected ? selected.name : "选择机器狗"} ▾`;
  }

  async function listRobots() {
    if (listingRobots) return;
    listingRobots = true;
    try {
      const response = await request("/api/robots", { cache: "no-store" });
      if (!response.ok) throw new Error("设备列表读取失败");
      const payload = response.payload;
      robots = payload.robots;
      if (!selectedRobot) {
        selectedRobot = payload.local_robot;
        robotGeneration++;
      }
      const selected = robots.find((robot) => robot.robot === selectedRobot);
      if (selected && streamAlive()) selected.connected = robotOnline;
      connectControl();
    } catch (_) {
      // Discovery is advisory. A missed HTTP reply cannot stop a healthy data channel.
      connectControl();
    } finally {
      listingRobots = false;
      updateControls();
      renderRobots();
    }
  }

  async function selectRobot(robot) {
    if (switchingRobot) return;
    if (robot === selectedRobot) {
      robotsDialog.close();
      return;
    }
    switchingRobot = true;
    updateControls();
    stop();
    const previous = selectedRobot;
    // Each request captures its robot. Late responses cannot change the UI
    // or forward a pending mode/held joystick to the newly selected robot.
    robotGeneration++;
    if (previous && robotOnline) {
      try {
        await request(api("command", previous), {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ axes: [0, 0, 0] }),
        });
        if (cameraEnabled) {
          await request(api("camera/toggle", previous), {
            method: "POST", headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ enabled: false }),
          });
        }
      } catch (_) { /* The previous controller also has its command watchdog. */ }
    }
    disconnectControl();
    selectedRobot = robot;
    sending = false;
    robotOnline = false;
    cameraEnabled = false;
    fpvPending = false;
    fpvRevision++;
    showFpv({ enabled: false, available: false, message: "正在读取图传状态" });
    cameraStream.removeAttribute("src");
    cameraStream.hidden = true;
    cameraPlaceholder.hidden = false;
    cameraToggle.textContent = "打开摄像头";
    cameraToggle.classList.remove("selected");
    cameraStatus.textContent = "摄像头关闭";
    controlSource = "web";
    selectedMode = "terrain";
    pendingMode = null;
    document.querySelectorAll(".action-button").forEach((button) => button.classList.remove("selected"));
    switchingRobot = false;
    renderRobots();
    updateControls();
    robotsDialog.close();
    await checkStatus();
    connectControl();
  }

  function vibrate(duration = 18) {
    if (navigator.vibrate) navigator.vibrate(duration);
  }

  function flash(button) {
    button.classList.add("pressed");
    window.setTimeout(() => button.classList.remove("pressed"), 160);
  }

  function setConnection(ok, text = ok ? (controlSource === "sbus" ? "遥控器控制" : "网页控制") : "未连接") {
    connection.classList.toggle("online", ok);
    connection.classList.toggle("offline", !ok);
    connection.querySelector("b").textContent = text;
  }

  function showAxes() {
    $("#axes").textContent = `${axes[0].toFixed(2)} / ${axes[1].toFixed(2)}`;
  }

  function activeLimits() {
    return modeLimits[selectedMode] || conservativeLimits;
  }

  function reverseLimit() {
    return activeLimits()[0];
  }

  async function enterLandscape() {
    rotateMessage.textContent = "正在切换横屏…";
    try {
      if (!document.fullscreenElement && document.documentElement.requestFullscreen) {
        await document.documentElement.requestFullscreen({ navigationUI: "hide" });
      }
      if (screen.orientation && screen.orientation.lock) {
        await screen.orientation.lock("landscape");
      }
      if (window.matchMedia("(orientation: portrait)").matches) {
        rotateMessage.textContent = "请关闭手机竖屏锁定，然后将手机横置";
      }
    } catch (_) {
      rotateMessage.textContent = "浏览器未允许自动旋转，请关闭竖屏锁定后横置手机";
    }
  }

  function updateKeyboardAxes() {
    const active = activeLimits();
    const forward = Number(heldKeys.has("KeyW")) - Number(heldKeys.has("KeyS"));
    axes[0] = forward * (forward < 0 ? reverseLimit() : active[0]);
    axes[1] = (heldKeys.has("KeyA") ? active[1] : 0) - (heldKeys.has("KeyD") ? active[1] : 0);
    axes[2] = (heldKeys.has("KeyQ") ? active[2] : 0) - (heldKeys.has("KeyE") ? active[2] : 0);
    showAxes();
  }

  function sendCommand(force = false) {
    if (!selectedRobot || switchingRobot || channel?.readyState !== "open") return;
    if (controlSource === "sbus") {
      axes.fill(0);
      pendingMode = null;
    }
    if (document.hidden || !robotOnline) axes.fill(0);
    if (channel.bufferedAmount > 4096) {
      disconnectControl();
      return;
    }
    try {
      channel.send(JSON.stringify({ axes: [...axes], seq: ++commandSequence }));
    } catch (_) {
      disconnectControl();
      return;
    }
    if (pendingMode && !sending) {
      const mode = pendingMode;
      pendingMode = null;
      sendMode(mode);
    }
  }

  async function sendMode(mode) {
    if (sending) return;
    sending = true;
    updateControls();
    const generation = robotGeneration;
    const target = selectedRobot;
    try {
      const response = await request(api("mode", target), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode }),
        cache: "no-store",
      }, 4000);
      if (generation !== robotGeneration) return;
      if (response.ok) {
        const result = response.payload;
        if (!result.accepted) $("#page-status").textContent = result.message;
      }
    } catch (_) {
      if (generation === robotGeneration) $("#page-status").textContent = "模式切换未确认，请重试";
    } finally {
      if (generation === robotGeneration) sending = false;
      updateControls();
    }
  }

  function showLight(enabled) {
    lightToggle.textContent = enabled ? "关闭白灯" : "打开白灯";
    lightToggle.classList.toggle("selected", enabled);
    lightToggle.setAttribute("aria-pressed", String(enabled));
  }

  async function toggleLight() {
    if (!robotOnline || switchingRobot) return;
    const generation = robotGeneration;
    lightToggle.disabled = true;
    try {
      const response = await request(api("light/toggle"), {
        method: "POST", headers: { "Content-Type": "application/json" }, body: "{}",
      });
      const payload = response.payload;
      if (generation !== robotGeneration) return;
      if (!response.ok || !payload.success) throw new Error(payload.message || "白灯切换失败");
      showLight(Boolean(payload.white_light_enabled));
    } catch (error) {
      if (generation === robotGeneration) $("#page-status").textContent = error.message;
    } finally {
      updateControls();
    }
  }

  function showFpv(state) {
    fpvRemote = state?.remote_controlled === true;
    fpvEnabled = fpvRemote ? state?.rc_enabled === true : state?.enabled === true;
    fpvAvailable = state?.available === true;
    fpvToggle.textContent = fpvPending ? "图传切换中…" : fpvEnabled ? "关闭图传" : "打开图传";
    if (fpvRemote && !fpvPending) fpvToggle.textContent = fpvEnabled ? "遥控器 A：图传开" : "遥控器 A：图传关";
    if (state?.control_enabled === false) fpvToggle.textContent = "图传暂未启用";
    fpvToggle.classList.toggle("selected", fpvEnabled);
    fpvToggle.setAttribute("aria-pressed", String(fpvEnabled));
    fpvToggle.title = state?.message || (fpvRemote ? "遥控器已连接，请通过 A 开关图传" : fpvAvailable ? "控制天空端图传进程" : "图传服务不可用");
    updateControls();
  }

  async function toggleFpv() {
    if (fpvToggle.disabled) return;
    const generation = robotGeneration;
    const target = selectedRobot;
    fpvPending = true;
    fpvRevision++;
    fpvToggle.textContent = "图传切换中…";
    updateControls();
    try {
      const response = await request(api("fpv/capture", target), {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: !fpvEnabled }),
      }, 5000);
      if (generation !== robotGeneration) return;
      if (typeof response.payload.enabled === "boolean") showFpv(response.payload);
      if (!response.ok || !response.payload.success)
        throw new Error(response.payload.message || response.payload.error || "图传切换失败");
    } catch (error) {
      if (generation === robotGeneration) $("#page-status").textContent = error.message;
    } finally {
      if (generation === robotGeneration) {
        fpvPending = false;
        fpvRevision++;
        fpvToggle.textContent = fpvEnabled ? "关闭图传" : "打开图传";
        updateControls();
        checkStatus();
      }
    }
  }

  async function checkStatus() {
    if (!selectedRobot || switchingRobot) return;
    const generation = robotGeneration;
    if (checkingStatus === generation) return;
    checkingStatus = generation;
    const revision = fpvRevision;
    try {
      const response = await request(api("status"), { cache: "no-store" });
      if (generation !== robotGeneration) return;
      if (response.ok) {
        const payload = response.payload;
        if (generation !== robotGeneration) return;
        if (payload.robot !== selectedRobot) return;
        const source = payload.control_source || "web";
        if (!streamAlive() && source !== controlSource) {
          axes.fill(0);
          heldKeys.clear();
          pendingMode = null;
          showAxes();
        }
        if (!streamAlive()) controlSource = source;
        updateControls();
        showLight(Boolean(payload.white_light_enabled));
        if (!fpvPending && revision === fpvRevision) showFpv(payload.fpv);
      } else {
        connectControl();
      }
    } catch (_) {
      if (generation === robotGeneration) {
        connectControl();
      }
    } finally {
      if (checkingStatus === generation) checkingStatus = null;
    }
  }

  function stop() {
    heldKeys.clear();
    axes.fill(0);
    showAxes();
    document.querySelectorAll(".joystick").forEach((stick) => {
      stick.dispatchEvent(new Event("go2-stick-reset"));
      stick.querySelector(".knob").style.transform = "translate(0px, 0px)";
      stick.classList.remove("active");
    });
    sendCommand(true);
  }

  function bindJoystick(element, kind) {
    const knob = element.querySelector(".knob");
    let pointer = null;

    function update(event) {
      const rect = element.getBoundingClientRect();
      const radius = rect.width * 0.34;
      let x = event.clientX - rect.left - rect.width / 2;
      let y = event.clientY - rect.top - rect.height / 2;
      if (kind === "yaw") y = 0;
      const length = Math.hypot(x, y);
      if (length > radius) {
        x = x / length * radius;
        y = y / length * radius;
      }
      knob.style.transform = `translate(${x}px, ${y}px)`;
      if (kind === "move") {
        const active = activeLimits();
        axes[0] = -y / radius * (y > 0 ? reverseLimit() : active[0]);
        axes[1] = -x / radius * active[1];
      } else {
        axes[2] = -x / radius * activeLimits()[2];
      }
      showAxes();
    }

    function release(event) {
      if (pointer === null || (event && event.pointerId !== pointer)) return;
      if (element.hasPointerCapture(pointer)) element.releasePointerCapture(pointer);
      pointer = null;
      element.classList.remove("active");
      knob.style.transform = "translate(0px, 0px)";
      if (kind === "move") axes[0] = axes[1] = 0;
      else axes[2] = 0;
      showAxes();
      sendCommand(true);
    }

    element.addEventListener("pointerdown", (event) => {
      if (!controlEnabled() || robotsDialog.open) return;
      event.preventDefault();
      pointer = event.pointerId;
      element.setPointerCapture(pointer);
      element.classList.add("active");
      vibrate(10);
      update(event);
    });
    element.addEventListener("pointermove", (event) => {
      if (!controlEnabled() || robotsDialog.open) return;
      if (event.pointerId === pointer) update(event);
    });
    element.addEventListener("pointerup", release);
    element.addEventListener("pointercancel", release);
    element.addEventListener("go2-stick-reset", () => release());
  }

  async function toggleCamera() {
    if (!robotOnline || switchingRobot) return;
    const generation = robotGeneration;
    flash(cameraToggle);
    try {
      const response = await request(api("camera/toggle"), {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ enabled: !cameraEnabled }),
        cache: "no-store",
      }, 9000);
      const payload = response.payload;
      if (generation !== robotGeneration) return;
      if (!response.ok) throw new Error(payload.error || "camera error");
      cameraEnabled = Boolean(payload.enabled);
      cameraToggle.textContent = cameraEnabled ? "关闭摄像头" : "打开摄像头";
      cameraToggle.classList.toggle("selected", cameraEnabled);
      cameraPlaceholder.hidden = cameraEnabled;
      cameraStream.hidden = !cameraEnabled;
      cameraStatus.textContent = cameraEnabled ? "摄像头已打开" : "摄像头关闭";
      if (cameraEnabled) {
        const player = new URL(window.location.href);
        player.hostname = payload.webrtc_host;
        player.port = String(payload.webrtc_port);
        player.pathname = "/camera";
        player.search = "?controls=false&muted=true&autoplay=true&playsInline=true";
        player.hash = "";
        cameraStream.src = player.href;
      } else cameraStream.removeAttribute("src");
    } catch (error) {
      if (generation === robotGeneration) cameraStatus.textContent = `摄像头打开失败：${error.message}`;
    }
  }

  tokenDialog.remove();

  /* v9's token dialog is intentionally not used by the trusted local simple UI.
  $("#connect-button").addEventListener("click", async (event) => {
    event.preventDefault();
    const candidate = tokenInput.value.trim();
    if (!candidate) {
      tokenError.textContent = "请输入访问码";
      return;
    }
    token = candidate;
    sessionStorage.setItem("go2-token", token);
    await checkStatus();
    if (token) {
      tokenError.textContent = "";
      tokenDialog.close();
      sendCommand();
    } else {
      tokenError.textContent = "访问码不正确";
    }
  }); */

  connection.addEventListener("click", () => {
    if (!robotOnline) { nextConnectAt = 0; connectControl(); }
  });
  landscapeButton.addEventListener("click", enterLandscape);
  robotsButton.addEventListener("click", () => {
    stop();
    renderRobots();
    robotsDialog.showModal();
    listRobots();
  });
  $("#robots-close").addEventListener("click", () => robotsDialog.close());
  document.querySelectorAll(".action-button").forEach((button) => {
    button.addEventListener("click", () => {
      if (button.disabled || !controlEnabled()) return;
      if (["navigation", "ai_agent", "passive", "passive_direct"].includes(button.dataset.mode)) {
        heldKeys.clear();
        axes.fill(0);
        showAxes();
      }
      selectedMode = button.dataset.mode || "terrain";
      updateKeyboardAxes();
      document.querySelectorAll(".action-button").forEach((item) => item.classList.remove("selected"));
      button.classList.add("selected");
      pendingMode = button.dataset.mode || button.dataset.fallback || "stand";
      vibrate(25);
      sendCommand(true);
    });
  });

  document.querySelectorAll(".page-button").forEach((button) => {
    button.addEventListener("click", () => {
      const page = Number(button.dataset.page);
      document.querySelectorAll(".page").forEach((item, index) => item.classList.toggle("active", index === page));
      document.querySelectorAll(".page-button").forEach((item) => item.classList.remove("selected"));
      button.classList.add("selected");
      $("#page-status").textContent = `第 ${page + 1} 页 · 键盘 Q/W/E/A/S/D 可控制移动`;
    });
  });

  cameraToggle.addEventListener("click", toggleCamera);
  lightToggle.addEventListener("click", toggleLight);
  fpvToggle.addEventListener("click", toggleFpv);

  bindJoystick($("#move-stick"), "move");
  bindJoystick($("#turn-stick"), "yaw");
  document.addEventListener("keydown", (event) => {
    if (!controlEnabled() || robotsDialog.open) return;
    if (!driveKeys.has(event.code) || event.ctrlKey || event.metaKey || event.altKey) return;
    event.preventDefault();
    heldKeys.add(event.code);
    updateKeyboardAxes();
    sendCommand();
  });
  document.addEventListener("keyup", (event) => {
    if (!driveKeys.has(event.code)) return;
    event.preventDefault();
    heldKeys.delete(event.code);
    updateKeyboardAxes();
    sendCommand(true);
  });
  window.addEventListener("blur", stop);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stop();
    else listRobots().then(checkStatus);
  });
  window.addEventListener("online", () => listRobots().then(checkStatus));
  window.addEventListener("pagehide", stop);

  showAxes();
  updateControls();
  listRobots().then(checkStatus);
  window.setInterval(sendCommand, 50);
  window.setInterval(checkStatus, 1000);
  window.setInterval(listRobots, 1000);
  window.setInterval(connectControl, 500);
})();
