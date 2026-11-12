(() => {
  'use strict';

  const SEND_MS = 50;          // 20 Hz HTTP joystick updates
  const STATUS_MS = 500;       // 2 Hz status polling
  const DEAD_ZONE = 0.08;

  const COMMAND_LABELS = {
    stand_up: '站立',
    sit_down: '趴下',
    balance_stand: '平衡',
    move_mode: '移动',
    lock: '锁定',
    slow: '慢速',
    normal: '正常速度',
    fast: '快速',
  };

  const el = {
    linkDot: document.getElementById('link-dot'),
    linkText: document.getElementById('link-text'),
    battery: document.getElementById('battery-pill'),
    mode: document.getElementById('mode-pill'),
    postureBar: document.getElementById('posture-bar'),
    speedBar: document.getElementById('speed-bar'),
    estop: document.getElementById('estop'),
    overlay: document.getElementById('lock-overlay'),
    lockTitle: document.getElementById('lock-title'),
    lockReason: document.getElementById('lock-reason'),
    resume: document.getElementById('resume-btn'),
    retry: document.getElementById('retry-btn'),
    toast: document.getElementById('toast'),
    stickLeft: document.getElementById('stick-left'),
    stickRight: document.getElementById('stick-right'),
  };

  const state = {
    connected: false,
    inControl: false,
    estop: false,
    stale: true,
    lastDriver: null,
    seq: 0,
    dirty: false,
    forceZero: false,
    inFlight: false,
    forward: 0,
    yaw: 0,
    strafe: 0,
    head: 0,
    leftActive: false,
    rightActive: false,
    controlsEnabled: false,
    estopPending: false,
  };

  let leftStick = null;
  let rightStick = null;
  let toastTimer = null;
  let resizeTimer = null;
  let statusTimer = null;

  function clamp(value, low, high) {
    return Math.max(low, Math.min(high, value));
  }

  function vibrate(pattern) {
    try {
      if (navigator.vibrate) {
        navigator.vibrate(pattern);
      }
    } catch (_) {
      /* haptics are optional */
    }
  }

  function showToast(text, duration = 1800) {
    el.toast.textContent = text;
    el.toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.toast.classList.remove('show'), duration);
  }

  function setLink(ok) {
    state.connected = ok;
    el.linkDot.classList.toggle('off', !ok);
    el.linkText.textContent = ok ? '已连接' : '未连接';
  }

  function setControlsEnabled(enabled, title, reason, showResume = false) {
    state.controlsEnabled = enabled;
    if (enabled) {
      el.overlay.hidden = true;
      el.resume.hidden = true;
    } else {
      el.overlay.hidden = false;
      el.lockTitle.textContent = title || '等待连接';
      el.lockReason.textContent = reason || '';
      el.resume.hidden = !showResume;
    }
    document.querySelectorAll('.cmd-btn, .speed-btn').forEach((button) => {
      button.setAttribute('aria-disabled', enabled ? 'false' : 'true');
    });
  }

  function updateAxes() {
    const left = leftStick ? leftStick.value : { x: 0, y: 0 };
    const right = rightStick ? rightStick.value : { x: 0, y: 0 };
    state.forward = clamp(-left.y, -1, 1);
    state.yaw = clamp(-left.x, -1, 1);
    state.strafe = clamp(-right.x, -1, 1);
    // 右摇杆只做左右平移；头部轴固定为 0。
    state.head = 0;
    state.leftActive = !!(leftStick && leftStick.active);
    state.rightActive = !!(rightStick && rightStick.active);
  }

  function sticksActive() {
    return state.leftActive || state.rightActive;
  }

  function onStickChange() {
    updateAxes();
    scheduleJoy(!sticksActive());
  }

  function scheduleJoy(forceZero = false) {
    state.dirty = true;
    if (forceZero) {
      state.forceZero = true;
    }
    pumpJoy();
  }

  async function pumpJoy() {
    if (state.inFlight) {
      return;
    }
    if (!state.connected) {
      return;
    }
    if (!state.inControl && !state.forceZero) {
      return;
    }
    if (!state.dirty && !sticksActive() && !state.forceZero) {
      return;
    }

    const forceZero = state.forceZero;
    state.forceZero = false;
    state.dirty = false;
    state.inFlight = true;
    state.seq += 1;

    const payload = forceZero
      ? { forward: 0, yaw: 0, strafe: 0, head: 0, seq: state.seq }
      : {
        forward: state.forward,
        yaw: state.yaw,
        strafe: state.strafe,
        head: state.head,
        seq: state.seq,
      };

    const result = await fetchJson('/api/joy', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
      keepalive: forceZero,
    }, 450);

    state.inFlight = false;
    if (!result.ok && forceZero) {
      showToast('归零请求发送失败，请检查手机与控制器连接');
    }
    if (state.dirty || state.forceZero) {
      pumpJoy();
    }
  }

  function zeroNow() {
    if (leftStick) {
      leftStick.release();
    }
    if (rightStick) {
      rightStick.release();
    }
    state.forward = 0;
    state.yaw = 0;
    state.strafe = 0;
    state.head = 0;
    state.leftActive = false;
    state.rightActive = false;
    scheduleJoy(true);
  }

  async function fetchJson(url, options = {}, timeoutMs = 1200) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(url, {
        ...options,
        signal: controller.signal,
        cache: 'no-store',
      });
      const text = await response.text();
      let data = null;
      if (text) {
        try {
          data = JSON.parse(text);
        } catch (_) {
          data = null;
        }
      }
      return { ok: response.ok, status: response.status, data };
    } catch (error) {
      return { ok: false, status: 0, data: null, error };
    } finally {
      clearTimeout(timer);
    }
  }

  function updateHud(driver, stale) {
    const battery = driver.battery || {};
    const power = Number(battery.power);
    const volt = Number(battery.volt);
    el.battery.textContent = Number.isFinite(power)
      ? `${power}% · ${Number.isFinite(volt) ? volt.toFixed(1) : '--'}V`
      : '--% · --V';

    let mode = '等待控制权';
    if (stale) {
      mode = '状态超时';
    } else if (!driver.connected) {
      mode = '未连接';
    } else if (driver.estop_latched) {
      mode = '急停锁定';
    } else if (driver.in_control) {
      const fn = String(driver.function_mode_name || '?').toUpperCase();
      const cm = String(driver.control_mode_name || '?').toUpperCase();
      mode = `${fn} · ${cm}`;
    }
    if (Number(driver.fault_count) > 0) {
      mode += ` · 故障${driver.fault_count}`;
    }
    el.mode.textContent = mode;
    el.mode.classList.toggle('danger', !!(driver.estop_latched || stale));
  }

  function updateSpeedButtons(driver) {
    const level = Number(driver.speed_level);
    const levels = { slow: 1, normal: 2, fast: 3 };
    el.speedBar.querySelectorAll('[data-command]').forEach((button) => {
      const selected = levels[button.dataset.command] === level;
      button.classList.toggle('selected', selected);
      button.setAttribute('aria-pressed', selected ? 'true' : 'false');
    });
  }

  function applyStatus(payload) {
    const driver = payload.driver;
    const stale = !!payload.stale;
    const connected = !!driver.connected && !stale;
    const estop = !!driver.estop_latched;
    const inControl = connected && !!driver.in_control && !estop;
    const wasInControl = state.inControl;

    state.stale = stale;
    state.inControl = inControl;
    state.estop = estop;
    state.lastDriver = driver;

    setLink(connected);
    updateHud(driver, stale);
    updateSpeedButtons(driver);

    if (state.estopPending && !estop) {
      setControlsEnabled(
        false,
        '急停指令已发送',
        '等待机器人确认急停状态。若长时间无响应，请使用实体遥控器或断电。',
        false);
      return;
    }

    if (!connected) {
      if (wasInControl || sticksActive()) {
        zeroNow();
      }
      setControlsEnabled(
        false,
        stale ? '状态超时' : '未连接机器狗',
        stale
          ? '驱动状态超过 2 秒未更新，请检查 launch 终端。'
          : '请检查机器狗电源、热点、驱动节点与手机网络。',
        false);
      return;
    }

    if (estop) {
      if (wasInControl || sticksActive()) {
        zeroNow();
      }
      setControlsEnabled(
        false,
        '急停已触发',
        driver.last_error || '机器人处于急停状态。确认场地安全后可尝试恢复。',
        true);
      return;
    }

    if (!inControl) {
      if (wasInControl || sticksActive()) {
        zeroNow();
      }
      setControlsEnabled(
        false,
        '正在获取控制权',
        driver.last_error ||
          '等待机器人确认 SDK 控制权；若被实体遥控器占用，请先释放实体遥控器。',
        false);
      return;
    }

    state.estopPending = false;
    setControlsEnabled(true);
  }

  async function pollStatus() {
    const result = await fetchJson('/api/status', {}, 1200);
    if (!result.ok || !result.data || !result.data.driver) {
      if (state.inControl || sticksActive()) {
        zeroNow();
      }
      state.inControl = false;
      state.estop = false;
      setLink(false);
      el.battery.textContent = '--% · --V';
      el.mode.textContent = '未连接';
      el.mode.classList.add('danger');
      setControlsEnabled(
        false,
        '未连接控制器',
        '无法获取驱动状态，请检查 launch 终端与手机网络。',
        false);
      return;
    }
    applyStatus(result.data);
  }

  async function sendCommand(name) {
    if (!state.controlsEnabled) {
      showToast('未取得控制权，命令未发送');
      return;
    }
    const result = await fetchJson('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: name }),
    }, 1200);
    if (!result.ok) {
      const reason = (result.data && result.data.error) || result.status;
      showToast(`命令发送失败: ${reason}`);
    } else {
      showToast(`已发送: ${COMMAND_LABELS[name] || name}`, 900);
    }
  }

  async function emergencyStop() {
    if (state.estopPending) {
      return;
    }
    state.estopPending = true;
    state.estop = true;
    state.inControl = false;
    zeroNow();
    vibrate([40, 30, 120]);
    setControlsEnabled(
      false,
      '急停已触发',
      '正在发送急停指令...',
      false);
    try {
      const result = await fetchJson('/api/estop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
        keepalive: true,
      }, 1200);
      if (!result.ok) {
        throw new Error(`HTTP ${result.status}`);
      }
      el.lockReason.textContent = '急停指令已发送，等待机器人确认...';
      showToast('急停指令已发送');
    } catch (error) {
      el.lockTitle.textContent = '急停发送失败';
      el.lockReason.textContent = '请立即使用实体遥控器或断开机器狗电源！';
      showToast('急停发送失败，请使用实体遥控器！', 4000);
    }
  }

  async function clearEstop() {
    state.estopPending = false;
    el.resume.hidden = true;
    setControlsEnabled(
      false,
      '正在恢复',
      '等待机器人站立并重新取得控制权...',
      false);
    const result = await fetchJson('/api/estop/clear', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    }, 1400);
    if (!result.ok) {
      showToast('恢复请求发送失败');
      setControlsEnabled(
        false,
        '恢复失败',
        '请检查连接，确认场地安全后重试。',
        true);
    }
  }

  function bindButtons() {
    [el.postureBar, el.speedBar].forEach((bar) => {
      bar.querySelectorAll('[data-command]').forEach((button) => {
        button.addEventListener('pointerdown', (event) => {
          event.preventDefault();
          if (!state.controlsEnabled) {
            showToast('未取得控制权');
            return;
          }
          const now = performance.now();
          const last = Number(button.dataset.lastPress || 0);
          if (now - last < 300) {
            return;
          }
          button.dataset.lastPress = String(now);
          vibrate(10);
          sendCommand(button.dataset.command);
        }, { passive: false });
      });
    });
  }

  function bindLifecycle() {
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        zeroNow();
      } else {
        pollStatus();
      }
    });
    window.addEventListener('pagehide', zeroNow);
    window.addEventListener('blur', zeroNow);
    window.addEventListener('orientationchange', zeroNow);
    window.addEventListener('resize', () => {
      clearTimeout(resizeTimer);
      resizeTimer = setTimeout(zeroNow, 100);
    });
    document.addEventListener('contextmenu', (event) => event.preventDefault());
    document.addEventListener('gesturestart', (event) => event.preventDefault());
  }

  function init() {
    leftStick = new window.Joystick(el.stickLeft, {
      onChange: onStickChange,
      deadZone: DEAD_ZONE,
    });
    rightStick = new window.Joystick(el.stickRight, {
      onChange: onStickChange,
      deadZone: DEAD_ZONE,
      axis: 'horizontal',
    });

    bindButtons();
    bindLifecycle();

    el.estop.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      emergencyStop();
    }, { passive: false });

    el.resume.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      clearEstop();
    }, { passive: false });

    el.retry.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      pollStatus();
    }, { passive: false });

    setControlsEnabled(false, '正在连接', '等待驱动节点状态...', false);
    pollStatus();
    statusTimer = setInterval(pollStatus, STATUS_MS);
    setInterval(() => {
      if (sticksActive()) {
        scheduleJoy(false);
      }
    }, SEND_MS);
  }

  window.addEventListener('DOMContentLoaded', init);
})();
