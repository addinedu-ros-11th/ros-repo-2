#!/usr/bin/env python3
import argparse
import heapq
import io
import json
import logging
import math
import os
import re
import socketserver
import threading
import time
import urllib.error
import urllib.request
import numpy as np
from datetime import datetime
from http import server
from urllib.parse import parse_qs, urlparse

# Camera runtime must stay isolated from ROS libcamera bindings.
_ros_py_paths = [p for p in list(os.sys.path) if "/opt/ros/jazzy/lib/python3.12/site-packages" in p]
_orig_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
_orig_libcamera_ipa = os.environ.get("LIBCAMERA_IPA_MODULE_PATH")
_local_site = "/usr/local/lib/aarch64-linux-gnu/python3.12/site-packages"
_system_site = "/usr/lib/aarch64-linux-gnu/python3.12/site-packages"
try:
    os.sys.path = [p for p in os.sys.path if "/opt/ros/jazzy/lib/python3.12/site-packages" not in p]
    for _p in (_system_site, _local_site):
        if _p in os.sys.path:
            os.sys.path.remove(_p)
    os.sys.path.insert(0, _system_site)
    os.sys.path.insert(0, _local_site)

    _ld_parts = [p for p in _orig_ld_library_path.split(":") if p and p != "/opt/ros/jazzy/lib"]
    os.environ["LD_LIBRARY_PATH"] = "/usr/local/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:" + ":".join(_ld_parts)
    os.environ["LIBCAMERA_IPA_MODULE_PATH"] = "/usr/local/lib/aarch64-linux-gnu/libcamera"

    from picamera2 import Picamera2
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput
    from libcamera import Transform
except Exception as _cam_exc:
    raise RuntimeError(f"Camera stack import failed: {_cam_exc}") from _cam_exc
finally:
    os.environ["LD_LIBRARY_PATH"] = _orig_ld_library_path
    if _orig_libcamera_ipa is None:
        os.environ.pop("LIBCAMERA_IPA_MODULE_PATH", None)
    else:
        os.environ["LIBCAMERA_IPA_MODULE_PATH"] = _orig_libcamera_ipa
    for _p in _ros_py_paths:
        if _p not in os.sys.path:
            os.sys.path.append(_p)

EMOTIONS = ["hello", "basic", "angry", "bored", "fun", "happy", "interest", "sad"]

PAGE = """\
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Pinky Camera</title>
  <style>
    :root { --bg: #f5f7fb; --panel: #ffffff; --text: #1f2937; --muted: #5f6b7a; --accent: #2b6cff; --joy: 140px; --panel-pad: 12px; }
    body { background: var(--bg); color: var(--text); font-family: sans-serif; text-align: center; margin: 0; }
    header { padding: 16px 12px 8px; font-size: 20px; font-weight: 700; }
    .wrap { max-width: 980px; margin: 0 auto; padding: 0 16px 20px; box-sizing: border-box; }
    .video-wrap { position: relative; display: block; width: 320px; max-width: 320px; margin: 0 auto; }
    img { width: 320px; height: 240px; max-width: 320px; max-height: 240px; border: 2px solid #cfd8e3; background: #fff; box-sizing: border-box; }
    #overlay { position: absolute; left: 0; top: 0; pointer-events: none; }
    .panel { margin: 14px auto 0; background: var(--panel); border: 1px solid #d5deea; padding: var(--panel-pad); }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 10px; }
    button { padding: 10px 8px; border: 1px solid #c7d2e0; background: #ffffff; color: var(--text); cursor: pointer; }
    button:hover { border-color: var(--accent); }
    .status { margin-top: 8px; margin-bottom: 8px; font-size: 12px; color: var(--muted); }
    .telemetry { position: absolute; top: 8px; left: 8px; display: grid; gap: 6px; text-align: left; }
    .video-meta { position: absolute; top: 8px; right: 8px; display: grid; gap: 6px; text-align: right; }
    .video-meta .card { background: transparent; border: none; padding: 0; font-size: 12px; text-shadow: 0 1px 2px rgba(0,0,0,0.7); text-align: right; }
    .video-meta .label { color: var(--muted); display: inline; margin-right: 6px; }
    .video-meta .value { display: inline; color: #fff; }
    .telemetry .card { background: transparent; border: none; padding: 0; font-size: 12px; text-shadow: 0 1px 2px rgba(0,0,0,0.7); text-align: left; }
    .telemetry .label { color: var(--muted); display: inline; margin-right: 6px; }
    .telemetry .value { display: inline; color: #fff; }
    .warn { color: #ff6b6b; }
    .ctrl { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 12px; align-items: center; }
    .joy-wrap { display: grid; place-items: center; }
    .joystick {
      width: var(--joy);
      height: var(--joy);
      border-radius: 50%;
      border: 2px solid #c7d2e0;
      background: #eef3fa;
      position: relative;
      touch-action: none;
      user-select: none;
    }
    .stick {
      width: 54px;
      height: 54px;
      border-radius: 50%;
      background: #ffffff;
      border: 2px solid #b8c4d6;
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%, -50%);
      transition: transform 0.05s linear;
    }
    .sliders { display: grid; gap: 8px; }
    .sliders label { font-size: 12px; color: var(--muted); text-align: left; }
    input[type=range] { width: 100%%; }
    .voice-panel { margin-top: 12px; padding: 0; display: grid; gap: 10px; box-sizing: border-box; width: 100%; }
    .voice-title { letter-spacing: 0.6px; font-size: 11px; color: var(--muted); text-align: center; }
    .voice-row { display: grid; grid-template-columns: minmax(0, 1fr) minmax(96px, var(--joy)); gap: 12px; align-items: center; width: 100%; box-sizing: border-box; }
    .voice-btn { width: 100%; min-width: 96px; padding: 10px 8px; border: 1px solid #c7d2e0; background: linear-gradient(135deg, #ffffff, #eef3fa); color: var(--text); cursor: pointer; font-size: 12px; font-weight: 700; letter-spacing: 0.3px; justify-self: end; }
    .voice-btn:hover { border-color: #9fb2cc; }
    .voice-btn.active { border-color: #6bff6b; color: #6bff6b; box-shadow: 0 0 0 1px #6bff6b inset; }
    .voice-meta { display: grid; gap: 6px; align-items: center; }
    .voice-status { font-size: 12px; color: var(--muted); text-align: left; }
    .voice-last { font-size: 12px; color: #475569; text-align: left; word-break: break-word; }
    .status.small { font-size: 11px; }
    .face-panel { margin-top: 12px; display: grid; gap: 8px; }
    .face-title { letter-spacing: 0.6px; font-size: 11px; color: var(--muted); text-align: center; }
    .face-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; justify-items: center; }
    .face-btn { width: 100%; padding: 10px 8px; border: 1px solid #c7d2e0; background: #ffffff; color: var(--text); cursor: pointer; font-size: 12px; font-weight: 700; letter-spacing: 0.3px; }
    .face-btn:hover { border-color: var(--accent); }
    .face-btn.active { border-color: #6bff6b; color: #6bff6b; box-shadow: 0 0 0 1px #6bff6b inset; }
    .face-btn.single { grid-column: 1 / -1; }
    input[type=number] { width: 100%%; padding: 10px 8px; border: 1px solid #c7d2e0; background: #ffffff; color: var(--text); box-sizing: border-box; }
    .num-stepper { display: grid; grid-template-columns: 52px minmax(72px, 1fr) 52px; gap: 8px; width: 100%; align-items: center; }
    .num-stepper .step-btn { width: 100%; min-height: 42px; font-size: 22px; font-weight: 700; line-height: 1; padding: 0; }
    .num-stepper input[type=number] { text-align: center; min-height: 42px; font-size: 20px; }
    @media (max-width: 900px) {
      header { font-size: 18px; padding: 12px 10px 6px; }
      .wrap { padding: 0 10px 16px; }
      .wrap { padding: 0 18px 16px; }
      img { width: 100%; }
      .panel { padding: 10px; }
      .grid { grid-template-columns: repeat(2, 1fr); gap: 10px; }
      button { padding: 14px 10px; font-size: 18px; }
      :root { --joy: 170px; }
      .joystick { width: var(--joy); height: var(--joy); }
      .stick { width: 64px; height: 64px; }
      .sliders label { font-size: 14px; }
      .status { font-size: 14px; }
      .voice-btn { width: 100%; padding: 14px 10px; font-size: 14px; }
      .voice-status, .voice-last { font-size: 14px; }
      .face-btn { width: 100%; padding: 14px 10px; font-size: 14px; }
    }
  </style>
</head>
<body>
  <header>Pinky MJPEG Stream</header>
  <div class="wrap">
    <div class="video-wrap">
      <img id="mjpeg" src="/stream.mjpg" />
      <canvas id="overlay"></canvas>
      <div class="telemetry">
        <div class="card">
          <span class="label">Battery(6.8~7.6)</span>
          <span id="battery" class="value">-- V</span>
        </div>
        <div class="card">
          <span class="label">Ultrasonic</span>
          <span id="us" class="value">-- m</span>
        </div>
        <div class="card">
          <span class="label">IR</span>
          <span id="ir" class="value">--</span>
        </div>
        <div class="card">
          <span class="label">LiDAR F/L/R/B</span>
          <span id="lidar-min" class="value">-- / -- / -- / -- m</span>
        </div>
        <div class="card">
          <span class="label">LiDAR State</span>
          <span id="lidar-state" class="value">--</span>
        </div>
      </div>
      <div class="video-meta">
        <div class="card">
          <span class="label">Res</span>
          <span class="value">__RES__</span>
        </div>
        <div class="card">
          <span class="label">FPS</span>
          <span class="value"><span id="fps-live">--</span> / __FPS__</span>
        </div>
      </div>
    </div>
    <div class="panel">
      <div class="ctrl">
        <div class="joy-wrap">
          <div class="joystick" id="joystick">
            <div class="stick" id="stick"></div>
          </div>
        </div>
        <div class="sliders">
          <label>Speed: <span id="speed-val">1.00</span></label>
          <input id="speed" type="range" min="0.30" max="1.50" step="0.05" value="1.00">
          <label>Turn: <span id="turn-val">2.00</span></label>
          <input id="turn" type="range" min="0.20" max="3.00" step="0.10" value="2.00">
        </div>
      </div>
      <div class="voice-panel">
        <div class="voice-title">Voice Control</div>
        <div class="voice-row">
          <div class="voice-meta">
            <div class="voice-status" id="voice-status">Voice idle</div>
            <div class="voice-last" id="voice-last">Last: --</div>
          </div>
          <button class="voice-btn" id="voice-btn">Voice start</button>
        </div>
      </div>
      <div class="face-panel">
        <div class="face-title">InsightFace</div>
        <div class="face-row">
          <button class="face-btn" id="face-register">Register Face</button>
          <button class="face-btn" id="face-delete">Delete Face</button>
        </div>
      </div>
      <div class="face-panel">
        <div class="face-title">Person Follow</div>
        <div class="face-row">
          <button class="face-btn single" id="autonav-toggle">Follow Start</button>
        </div>
      </div>
      <div class="face-panel">
        <div class="face-title">Orange Lane (Phase1)</div>
        <div class="face-row">
          <button class="face-btn" id="phase1-start">Phase1 Start</button>
          <button class="face-btn" id="phase1-stop">Phase1 Stop</button>
        </div>
      </div>
      <div class="face-panel">
        <div class="face-title">Parking (Phase2)</div>
        <div class="face-row">
          <button class="face-btn" id="phase2-start">Phase2 Start</button>
          <button class="face-btn" id="phase2-stop">Phase2 Stop</button>
        </div>
        <div class="face-row">
          <div class="num-stepper">
            <button class="face-btn step-btn" id="phase2-slot-dec">-</button>
            <input id="phase2-slot" type="number" min="1" max="10" step="1" value="5" />
            <button class="face-btn step-btn" id="phase2-slot-inc">+</button>
          </div>
          <button class="face-btn" id="phase2-reset-start">Reset + Start</button>
        </div>
      </div>
      <div class="face-panel">
        <div class="face-title">Digit Nav (ArUco)</div>
        <div class="face-row">
          <button class="face-btn" id="digit-go-1">Go #1</button>
          <button class="face-btn" id="digit-go-2">Go #2</button>
          <input id="digit-id" type="number" min="0" max="49" step="1" value="1" />
          <button class="face-btn" id="digit-go-id">Go ID</button>
          <button class="face-btn single" id="digit-stop">Digit Stop</button>
        </div>
        <div class="status small" id="digit-status">Digit: idle</div>
      </div>
      <div class="face-panel">
        <div class="face-title">Number Nav (1~4)</div>
        <div class="face-row">
          <button class="face-btn" id="number-go-1">Num 1</button>
          <button class="face-btn" id="number-go-2">Num 2</button>
          <button class="face-btn" id="number-go-3">Num 3</button>
          <button class="face-btn" id="number-go-4">Num 4</button>
          <button class="face-btn single" id="number-stop">Number Stop</button>
        </div>
        <div class="status small" id="number-status">Number: idle</div>
      </div>
      <div class="status" id="status">Ready</div>
    </div>
  </div>

  <script>
    const statusEl = document.getElementById('status');
    const speedEl = document.getElementById('speed');
    const turnEl = document.getElementById('turn');
    const speedVal = document.getElementById('speed-val');
    const turnVal = document.getElementById('turn-val');
    const joystick = document.getElementById('joystick');
    const stick = document.getElementById('stick');
    const batteryEl = document.getElementById('battery');
    const usEl = document.getElementById('us');
    const irEl = document.getElementById('ir');
    const lidarMinEl = document.getElementById('lidar-min');
    const lidarStateEl = document.getElementById('lidar-state');
    const voiceBtn = document.getElementById('voice-btn');
    const voiceStatus = document.getElementById('voice-status');
    const voiceLast = document.getElementById('voice-last');
    const faceRegisterBtn = document.getElementById('face-register');
    const faceDeleteBtn = document.getElementById('face-delete');
    const autoNavToggleBtn = document.getElementById('autonav-toggle');
    const phase1StartBtn = document.getElementById('phase1-start');
    const phase1StopBtn = document.getElementById('phase1-stop');
    const phase2SlotEl = document.getElementById('phase2-slot');
    const phase2SlotDecBtn = document.getElementById('phase2-slot-dec');
    const phase2SlotIncBtn = document.getElementById('phase2-slot-inc');
    const phase2StartBtn = document.getElementById('phase2-start');
    const phase2StopBtn = document.getElementById('phase2-stop');
    const phase2ResetStartBtn = document.getElementById('phase2-reset-start');
    const digitGo1Btn = document.getElementById('digit-go-1');
    const digitGo2Btn = document.getElementById('digit-go-2');
    const digitIdEl = document.getElementById('digit-id');
    const digitGoIdBtn = document.getElementById('digit-go-id');
    const digitStopBtn = document.getElementById('digit-stop');
    const digitStatusEl = document.getElementById('digit-status');
    const numberGo1Btn = document.getElementById('number-go-1');
    const numberGo2Btn = document.getElementById('number-go-2');
    const numberGo3Btn = document.getElementById('number-go-3');
    const numberGo4Btn = document.getElementById('number-go-4');
    const numberStopBtn = document.getElementById('number-stop');
    const numberStatusEl = document.getElementById('number-status');
    const imgEl = document.getElementById('mjpeg');
    const fpsLive = document.getElementById('fps-live');
    const canvas = document.getElementById('overlay');
    const ctx = canvas.getContext('2d');
    const floorAnalyzeCanvas = document.createElement('canvas');
    const floorAnalyzeCtx = floorAnalyzeCanvas.getContext('2d', { willReadFrequently: true });
    const floorMaskCanvas = document.createElement('canvas');
    const floorMaskCtx = floorMaskCanvas.getContext('2d');
    floorAnalyzeCanvas.width = 320;
    floorAnalyzeCanvas.height = 240;
    floorMaskCanvas.width = 320;
    floorMaskCanvas.height = 240;

    const resizeOverlay = () => {
      const rect = imgEl.getBoundingClientRect();
      canvas.style.width = rect.width + 'px';
      canvas.style.height = rect.height + 'px';
      canvas.width = Math.round(rect.width);
      canvas.height = Math.round(rect.height);
    };

    imgEl.addEventListener('load', () => {
      resizeOverlay();
    });
    window.addEventListener('resize', resizeOverlay);
    setTimeout(resizeOverlay, 500);

    const setStatus = (msg) => { statusEl.textContent = msg; };
    const setDigitStatus = (msg) => { digitStatusEl.textContent = msg; };
    const setNumberStatus = (msg) => { numberStatusEl.textContent = msg; };


    let moveTimer = null;
    let moveLin = 0;
    let moveAng = 0;
    let dragging = false;
    let latched = false;
    let lastMoveFeedback = '';
    let lastMoveFeedbackTs = 0;

    const formatMoveFeedback = (msg) => {
      const t = (msg || '').trim();
      if (!t) return '';
      if (t.startsWith('SAFE_STOP(')) {
        const why = t.slice('SAFE_STOP('.length, -1);
        return `안전정지: ${why}`;
      }
      if (t.startsWith('SAFE_SLOW(')) {
        const why = t.slice('SAFE_SLOW('.length, -1);
        return `감속주행: ${why}`;
      }
      if (t === 'ROS2 not available') return 'ROS2 연결이 없습니다.';
      return t;
    };

    const pushMoveFeedback = (msg, minIntervalMs = 700) => {
      const now = Date.now();
      const t = formatMoveFeedback(msg);
      if (!t) return;
      if (t !== lastMoveFeedback || (now - lastMoveFeedbackTs) > minIntervalMs) {
        setStatus(t);
        lastMoveFeedback = t;
        lastMoveFeedbackTs = now;
      }
    };

    const hardStopLocal = () => {
      moveLin = 0;
      moveAng = 0;
      if (moveTimer) {
        clearInterval(moveTimer);
        moveTimer = null;
      }
      centerStick();
    };

    const sendMove = async (lin, ang) => {
      try {
        const res = await fetch(`/cmd_vel?lin=${lin.toFixed(2)}&ang=${ang.toFixed(2)}`);
        const msg = (await res.text()).trim();
        if (!res.ok) {
          pushMoveFeedback(msg || `HTTP ${res.status}`, 400);
          return;
        }
        if (msg && msg !== 'OK') {
          pushMoveFeedback(msg, 250);
          if (msg.startsWith('SAFE_STOP(')) {
            stopFollowMode();
            hardStopLocal();
          }
        }
      } catch (e) {
        setStatus(`Move error: ${e}`);
      }
    };

    const startMove = (lin, ang) => {
      if (autoNavActive) {
        // Manual joystick overrides server-side autonomous mode.
        setAutoNavUi(false, '');
        fetch('/autonav/stop').catch(() => {});
      }
      moveLin = lin;
      moveAng = ang;
      if (moveTimer) return;
      sendMove(moveLin, moveAng);
      moveTimer = setInterval(() => sendMove(moveLin, moveAng), 150);
    };

    const stopMove = () => {
      moveLin = 0;
      moveAng = 0;
      if (moveTimer) {
        clearInterval(moveTimer);
        moveTimer = null;
      }
      sendMove(0, 0);
    };

    const centerStick = () => {
      stick.style.transform = 'translate(-50%, -50%)';
    };

    const handleMove = (clientX, clientY) => {
      const rect = joystick.getBoundingClientRect();
      const cx = rect.left + rect.width / 2;
      const cy = rect.top + rect.height / 2;
      let dx = clientX - cx;
      let dy = clientY - cy;
      const radius = rect.width / 2 - 10;
      const dist = Math.hypot(dx, dy);
      if (dist > radius) {
        dx = (dx / dist) * radius;
        dy = (dy / dist) * radius;
      }
      const nx = dx / radius; // -1..1
      const ny = dy / radius; // -1..1
      // forward is negative y on screen
      const lin = -ny * parseFloat(speedEl.value);
      const ang = -nx * parseFloat(turnEl.value);
      startMove(lin, ang);
      stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    };

    joystick.addEventListener('pointerdown', (e) => {
      dragging = true;
      latched = true;
      joystick.setPointerCapture(e.pointerId);
      handleMove(e.clientX, e.clientY);
    });
    joystick.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      handleMove(e.clientX, e.clientY);
    });
    joystick.addEventListener('pointerup', (e) => {
      dragging = false;
      joystick.releasePointerCapture(e.pointerId);
      centerStick();
    });
    joystick.addEventListener('pointercancel', () => {
      dragging = false;
      latched = false;
      stopMove();
      centerStick();
    });

    // Tap joystick center to stop when not dragging
    joystick.addEventListener('click', (e) => {
      if (dragging) return;
      latched = false;
      stopMove();
      centerStick();
    });

    speedEl.addEventListener('input', () => { speedVal.textContent = speedEl.value; });
    turnEl.addEventListener('input', () => { turnVal.textContent = turnEl.value; });

    // Voice control (runs on the phone browser via Web Speech API)
    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    let recognizer = null;
    let voiceActive = false;
    let voiceWanted = false;
    let voiceCooldown = false;
    let lastVoiceCmd = '';
    let lastVoiceCmdTs = 0;
    let chatBusy = false;
    let lastChatText = '';
    let lastChatTs = 0;
    let followLastAckTs = 0;
    let followActive = false;
    let followTimer = null;
    let conversationFollowUntil = 0;
    let autoNavActive = false;
    let autoNavMode = '';
    let latestBestFace = null;
    let latestAutoNav = null;
    let latestTelemetry = {
      battery: null,
      us: null,
      ir: null,
      pose: null,
      objects: null,
      obj_size: null,
      faces: null,
      face_size: null,
      fps: null,
    };
    const setVoiceStatus = (msg) => { voiceStatus.textContent = msg; };
    const setVoiceLast = (msg) => { voiceLast.textContent = `Last: ${msg}`; };
    const namePatterns = [
      /(?:제\s*이름은|내\s*이름은)\s*([가-힣A-Za-z]{2,20})\s*(?:입니다|이에요|예요|이야|야)?/i,
      /(?:저는|나는|난)\s*([가-힣A-Za-z]{2,20})\s*(?:입니다|이에요|예요|이야|야)?/i,
      /([가-힣A-Za-z]{2,20})\s*(?:라고\s*불러줘|로\s*불러줘)/i,
    ];

    const normalizeSpokenName = (name) => {
      let n = (name || '').trim();
      n = n.replace(/[.,!?~"']/g, '');
      // Remove common Korean copula/endings from spoken introduction.
      n = n.replace(/(입니다|이에요|예요|라고|이야|야)$/u, '').trim();
      // Keep only Korean/English letters.
      n = n.replace(/[^가-힣A-Za-z]/g, '');
      if (n.length < 2 || n.length > 20) return null;
      return n;
    };

    const extractNameFromSpeech = (text) => {
      const raw = (text || '').trim();
      if (!raw) return null;
      for (const p of namePatterns) {
        const m = raw.match(p);
        if (m && m[1]) {
          return normalizeSpokenName(m[1]);
        }
      }
      return null;
    };

    const registerFaceFromVoice = async (name) => {
      if (!name) return false;
      setStatus(`${name} 님 등록 중...`);
      try {
        const res = await fetch(`/face/register?name=${encodeURIComponent(name)}`);
        const text = await res.text();
        if (res.ok) {
          setStatus(`${name} 님, 등록되었습니다.`);
          return true;
        }
        setStatus(`이름 등록 실패: ${text}`);
      } catch (e) {
        setStatus(`이름 등록 실패: ${e}`);
      }
      return false;
    };

    const hasTTS = ('speechSynthesis' in window) && ('SpeechSynthesisUtterance' in window);
    let latestRecognizedName = '';
    let greetedNameInVoiceSession = '';
    const wakeWords = ['핑키', '핑키야', 'pinky'];
    let ttsVoices = [];
    const refreshTtsVoices = () => {
      try {
        ttsVoices = (window.speechSynthesis && window.speechSynthesis.getVoices) ? (window.speechSynthesis.getVoices() || []) : [];
      } catch (_) {
        ttsVoices = [];
      }
    };
    if (hasTTS) {
      refreshTtsVoices();
      if (window.speechSynthesis && typeof window.speechSynthesis.onvoiceschanged !== 'undefined') {
        window.speechSynthesis.onvoiceschanged = () => { refreshTtsVoices(); };
      }
    }

    const normRole = (role) => {
      const r = (role || '').toString().trim().toLowerCase();
      if (r === 'gate') return 'gate';
      return 'pinky';
    };

    const normVoice = (voiceHint) => {
      const v = (voiceHint || '').toString().trim().toLowerCase();
      if (v === 'male' || v === 'female' || v === 'auto') return v;
      return '';
    };

    const pickVoice = (role, voiceHint) => {
      refreshTtsVoices();
      if (!ttsVoices.length) return null;
      const vHint = normVoice(voiceHint);
      const target = vHint || (role === 'gate' ? 'male' : 'female');
      const ko = ttsVoices.filter(v => ((v.lang || '').toLowerCase().startsWith('ko')));
      const pool = ko.length ? ko : ttsVoices;
      const maleKeys = ['male', 'man', '남', '남성'];
      const femaleKeys = ['female', 'woman', '여', '여성'];
      const keys = target === 'male' ? maleKeys : femaleKeys;
      const found = pool.find(v => {
        const s = `${v.name || ''} ${v.voiceURI || ''}`.toLowerCase();
        return keys.some(k => s.includes(k));
      });
      return found || (pool[0] || null);
    };

    const speakText = (text, opts = {}) => {
      if (!hasTTS || !text) return;
      const role = normRole(opts.role);
      const voiceHint = normVoice(opts.voice);
      const u = new SpeechSynthesisUtterance(text);
      u.lang = 'ko-KR';
      u.rate = (role === 'gate') ? 0.95 : 1.0;
      u.pitch = (role === 'gate') ? 0.85 : 1.08;
      const v = pickVoice(role, voiceHint);
      if (v) u.voice = v;
      window.speechSynthesis.cancel();
      window.speechSynthesis.speak(u);
    };

    const speakGreetingForName = (name) => {
      if (!hasTTS || !name) return;
      speakText(`안녕하세요. ${name}님`, { role: 'pinky', voice: 'female' });
    };

    // Parking server announcement polling:
    // - First poll only syncs latest id without speaking old messages.
    // - Next polls speak only newly published announcement.
    let announceReady = false;
    let lastAnnounceId = 0;
    const pollAnnouncement = async () => {
      try {
        const res = await fetch(`/announce/latest?since=${lastAnnounceId}`);
        if (!res.ok) return;
        const data = await res.json();
        if (!data || !data.ok) return;
        const id = Number(data.id || 0);
        const text = (data.text || '').toString().trim();
        if (!announceReady) {
          announceReady = true;
          lastAnnounceId = id;
          return;
        }
        if (id > lastAnnounceId && text) {
          lastAnnounceId = id;
          setStatus(`안내: ${text}`);
          speakText(text, { role: data.role || 'gate', voice: data.voice || 'male' });
        }
      } catch (_) {}
    };
    setTimeout(pollAnnouncement, 800);
    setInterval(pollAnnouncement, 1200);

    const speakFollowAck = () => {
      if (!hasTTS) return;
      const targetName = latestRecognizedName && latestRecognizedName.trim();
      const msg = targetName ? `네, ${targetName}님 따라갈게요.` : '네, 따라갈게요.';
      speakText(msg, { role: 'pinky', voice: 'female' });
    };

    const callNavApi = async (path, doSpeak = true) => {
      try {
        const res = await fetch(path);
        const msg = (await res.text()).trim() || (res.ok ? 'OK' : 'failed');
        setStatus(msg);
        if (doSpeak && hasTTS) speakText(msg, { role: 'pinky', voice: 'female' });
        return res.ok;
      } catch (e) {
        setStatus(`NAV 오류: ${e}`);
        return false;
      }
    };

    const setAutoNavUi = (enabled, mode = '') => {
      autoNavActive = !!enabled;
      autoNavMode = (mode || '').toString();
      const isFollowMode = autoNavActive && (autoNavMode === 'person_follow' || autoNavMode === 'person_companion');
      const isPhase1Mode = autoNavActive && (autoNavMode === 'orange_lane_phase1');
      const isPhase2Mode = autoNavActive && (autoNavMode === 'parking_phase2');
      autoNavToggleBtn.classList.toggle('active', isFollowMode);
      autoNavToggleBtn.textContent = isFollowMode ? `Follow Stop (${autoNavMode})` : 'Follow Start';
      phase1StartBtn.classList.toggle('active', isPhase1Mode);
      phase1StartBtn.textContent = isPhase1Mode ? 'Phase1 Running' : 'Phase1 Start';
      phase1StopBtn.classList.toggle('active', isPhase1Mode);
      phase2StartBtn.classList.toggle('active', isPhase2Mode);
      phase2StartBtn.textContent = isPhase2Mode ? 'Phase2 Running' : 'Phase2 Start';
      phase2StopBtn.classList.toggle('active', isPhase2Mode);
    };

    const fetchAutoNavStatus = async (notify = true) => {
      try {
        const res = await fetch('/autonav/status');
        const data = await res.json();
        if (data && typeof data === 'object') {
          setAutoNavUi(!!data.enabled, data.mode || '');
          if (notify) {
            const age = (typeof data.last_seen_age === 'number') ? `${data.last_seen_age.toFixed(1)}s` : '--';
            setStatus(`AutoNav: mode=${data.mode || '--'} msg=${data.last_msg || '--'} seen=${age}`);
          }
          return true;
        }
      } catch (e) {
        if (notify) setStatus(`AutoNav 상태조회 실패: ${e}`);
      }
      return false;
    };

    const callAutoNavApi = async (path, doSpeak = false) => {
      try {
        const res = await fetch(path);
        const msg = (await res.text()).trim() || (res.ok ? 'OK' : 'failed');
        if (res.ok && path.includes('/autonav/start')) {
          const m = path.match(/mode=([^&]+)/);
          const mode = m ? decodeURIComponent(m[1]) : '';
          setAutoNavUi(true, mode);
        }
        if (res.ok && path.includes('/autonav/stop')) {
          setAutoNavUi(false, '');
        }
        await fetchAutoNavStatus(false);
        setStatus(msg);
        if (doSpeak && hasTTS) speakText(msg, { role: 'pinky', voice: 'female' });
        return res.ok;
      } catch (e) {
        setStatus(`AutoNav 오류: ${e}`);
        return false;
      }
    };

    const buildSceneSummary = () => {
      const t = latestTelemetry || {};
      const parts = [];

      const poseBoxes = (t.pose && Array.isArray(t.pose.boxes)) ? t.pose.boxes : [];
      const faces = Array.isArray(t.faces) ? t.faces : [];
      let personCount = Math.max(poseBoxes.length, faces.length);

      if (personCount > 0) {
        parts.push(`사람 ${personCount}명이 보입니다`);
      } else {
        parts.push('사람은 보이지 않습니다');
      }

      const known = faces
        .filter((f) => f && f.name && f.name !== 'unknown')
        .sort((a, b) => (b.score || 0) - (a.score || 0))
        .map((f) => f.name);
      const uniqueKnown = [...new Set(known)];
      if (uniqueKnown.length === 1) {
        parts.push(`${uniqueKnown[0]}님으로 인식됩니다`);
      } else if (uniqueKnown.length > 1) {
        parts.push(`${uniqueKnown.slice(0, 2).join(', ')}님이 인식됩니다`);
      }

      if (typeof t.us === 'number') {
        parts.push(`앞쪽 거리는 약 ${t.us.toFixed(2)}미터입니다`);
        if (t.us < 0.35) {
          parts.push('가까운 장애물이 있습니다');
        }
      }
      if (typeof t.battery === 'number') {
        parts.push(`배터리는 ${t.battery.toFixed(2)}볼트입니다`);
      }

      const objects = Array.isArray(t.objects) ? t.objects : [];
      if (objects.length > 0) {
        const counts = {};
        objects.forEach((o) => {
          const label = (o && o.label) ? String(o.label).toLowerCase() : '';
          if (!label || label === 'person') return;
          counts[label] = (counts[label] || 0) + 1;
        });
        const ranked = Object.entries(counts).sort((a, b) => b[1] - a[1]).slice(0, 3);
        if (ranked.length > 0) {
          const itemText = ranked
            .map(([k, v]) => `${k} ${v}개`)
            .join(', ');
          parts.push(`사물은 ${itemText}가 보입니다`);
        }
      }

      return `${parts.join('. ')}.`;
    };

    const isSceneQuery = (rawText) => {
      const t = (rawText || '').replace(/\s+/g, '');
      return (
        t.includes('뭐보여') ||
        t.includes('뭐가보여') ||
        t.includes('뭐가보이나') ||
        t.includes('뭐가보이냐') ||
        t.includes('뭐가보이니') ||
        t.includes('뭐가보이네') ||
        t.includes('무엇이보여') ||
        t.includes('무엇이보이나') ||
        t.includes('무엇이보이냐') ||
        t.includes('무엇이보이니') ||
        t.includes('화면설명') ||
        t.includes('주변설명') ||
        t.includes('상황설명') ||
        t.includes('보이는거설명') ||
        t.includes('장면설명')
      );
    };

    const stopFollowMode = () => {
      followActive = false;
      if (followTimer) {
        clearInterval(followTimer);
        followTimer = null;
      }
    };

    const startFollowMode = () => {
      followActive = true;
      if (followTimer) return;
      followTimer = setInterval(() => {
        if (!followActive) return;
        const f = latestBestFace;
        if (!f || !f.box || !f.size || !f.size.w || !f.size.h) {
          startMove(0, 0);
          return;
        }
        const box = f.box;
        const cx = (box[0] + box[2]) * 0.5;
        const centerNorm = (cx / f.size.w) - 0.5; // -0.5(left) .. +0.5(right)
        const deadband = 0.08;
        let err = centerNorm;
        if (Math.abs(err) < deadband) err = 0;
        const baseSpeed = parseFloat(speedEl.value);
        const turnGain = parseFloat(turnEl.value);
        const ang = -err * 2.0 * turnGain;
        const linScale = Math.max(0.15, 1.0 - Math.min(1.0, Math.abs(err) * 2.2));
        const lin = baseSpeed * 0.65 * linScale;
        startMove(lin, ang);
      }, 150);
    };

    const keepCompanionFollow = (ms = 9000) => {
      const now = Date.now();
      conversationFollowUntil = Math.max(conversationFollowUntil, now + Math.max(1000, ms));
      if (autoNavActive || !latestBestFace) return;
      startFollowMode();
    };

    const extractWakeCommandText = (text) => {
      const raw = (text || '').trim();
      if (!raw) return null;
      const lower = raw.toLowerCase();
      if (!wakeWords.some((w) => lower.includes(w))) return null;
      // Treat as command mode only when wake-word appears at the sentence head.
      // This avoids misclassifying normal conversation like "핑키라는 로봇..." as a command.
      if (!/^\s*(핑키야?|pinky)(?:\s|$|[,:.!?])/i.test(raw)) return null;
      // Strip wake words and common separators to isolate command phrase.
      let cmd = raw
        .replace(/^\s*(핑키야?|pinky)\s*/i, ' ')
        .replace(/[,:.!?]/g, ' ')
        .replace(/\s+/g, ' ')
        .trim();
      return cmd || '';
    };

    const applyVoiceCommand = (text) => {
      const t = text.replace(/\s+/g, '');
      const parseParkingSlot = (raw) => {
        const compact = (raw || '').replace(/\s+/g, '').toLowerCase();
        const hasParkingWord = (
          compact.includes('주차') ||
          compact.includes('파킹') ||
          compact.includes('parking')
        );
        if (!hasParkingWord) return null;
        const m1 = compact.match(/(10|[1-9])번/);
        if (m1) return Math.max(1, Math.min(10, parseInt(m1[1], 10)));
        const m2 = compact.match(/(?:slot|슬롯|주차면)(10|[1-9])/);
        if (m2) return Math.max(1, Math.min(10, parseInt(m2[1], 10)));
        const m3 = compact.match(/(10|[1-9])/);
        if (m3) return Math.max(1, Math.min(10, parseInt(m3[1], 10)));
        return null;
      };
      const now = Date.now();
      const speed = parseFloat(speedEl.value);
      const turn = parseFloat(turnEl.value);
      const speedMin = parseFloat(speedEl.min);
      const speedMax = parseFloat(speedEl.max);
      const speedStep = parseFloat(speedEl.step) || 0.05;
      const setSpeed = (v) => {
        const nv = Math.min(speedMax, Math.max(speedMin, v));
        speedEl.value = nv.toFixed(2);
        speedVal.textContent = speedEl.value;
        return nv;
      };
      const slot = parseParkingSlot(t);
      if (slot !== null) {
        stopFollowMode();
        stopMove();
        callDigitApi('/digit/stop');
        callNumberApi('/number/stop');
        if (phase2SlotEl) phase2SlotEl.value = String(slot);
        callAutoNavApi(`/autonav/start?mode=parking_phase2&slot=${slot}`);
        return `park-slot-${slot}`;
      }
      if (
        t.includes('출발점저장') ||
        t.includes('출발지저장') ||
        t.includes('홈저장') ||
        t.includes('집저장')
      ) {
        callNavApi('/nav/home/set');
        return 'home-set';
      }
      if (t.includes('복귀중지') || t.includes('돌아오기중지')) {
        callNavApi('/nav/return/stop');
        return 'return-stop';
      }
      if (
        t.includes('자율주행중지') ||
        t.includes('자율주행정지') ||
        t.includes('오토중지') ||
        t.includes('자동주행중지') ||
        t.includes('autonavstop')
      ) {
        callAutoNavApi('/autonav/stop');
        return 'autonav-stop';
      }
      if (t.includes('정지') || t.includes('멈춰') || t.includes('스톱') || t.includes('stop')) {
        stopFollowMode();
        stopMove();
        callAutoNavApi('/autonav/stop', false);
        return 'stop';
      }
      if (
        t.includes('자율주행시작') ||
        t.includes('자율주행') ||
        t.includes('오토주행') ||
        t.includes('자동주행') ||
        t.includes('오토파일럿') ||
        t.includes('autonav')
      ) {
        stopFollowMode();
        stopMove();
        callAutoNavApi('/autonav/start?mode=person_follow');
        return 'autonav-start';
      }
      if (t.includes('돌아와') || t.includes('복귀') || t.includes('집으로가')) {
        stopFollowMode();
        stopMove();
        callNavApi('/nav/return/start');
        return 'return-home';
      }
      if (t.includes('따라와') || t.includes('따라와줘') || t.includes('따라오기') || t.includes('followme')) {
        stopFollowMode();
        stopMove();
        callAutoNavApi('/autonav/start?mode=person_follow', false);
        if ((now - followLastAckTs) > 2500) {
          speakFollowAck();
          followLastAckTs = now;
        }
        return 'follow';
      }
      if (isSceneQuery(t)) {
        const summary = buildSceneSummary();
        setStatus(summary);
        speakText(summary);
        return 'scene';
      }
      if (t.includes('빠르게') || t.includes('속도올려') || t.includes('speedup')) {
        const nv = setSpeed(speed + speedStep);
        return `speed ${nv.toFixed(2)}`;
      }
      if (t.includes('천천히') || t.includes('느리게') || t.includes('속도내려') || t.includes('slow')) {
        const nv = setSpeed(speed - speedStep);
        return `speed ${nv.toFixed(2)}`;
      }
      if (t.includes('앞') || t.includes('전진') || t.includes('forward') || t.includes('go')) {
        stopFollowMode();
        startMove(speed, 0);
        return 'forward';
      }
      if (t.includes('뒤') || t.includes('후진') || t.includes('back')) {
        stopFollowMode();
        startMove(-speed, 0);
        return 'back';
      }
      if (t.includes('좌회전') || t.includes('좌') || t.includes('왼') || t.includes('left')) {
        stopFollowMode();
        startMove(0, turn);
        return 'left';
      }
      if (t.includes('우회전') || t.includes('우') || t.includes('오') || t.includes('right')) {
        stopFollowMode();
        startMove(0, -turn);
        return 'right';
      }
      return 'unknown';
    };

    const shouldRunVoiceCmd = (cmd) => {
      const now = Date.now();
      const debounceMs = (cmd === 'follow' || cmd === 'scene') ? 2500 : 700;
      // Allow repeating same command after a short interval (useful for stop/forward loops).
      if (cmd !== lastVoiceCmd || (now - lastVoiceCmdTs) > debounceMs) {
        lastVoiceCmd = cmd;
        lastVoiceCmdTs = now;
        return true;
      }
      return false;
    };

    const shouldRunChat = (text) => {
      const t = (text || '').trim();
      if (t.length < 2) return false;
      const now = Date.now();
      if (t === lastChatText && (now - lastChatTs) < 2500) return false;
      lastChatText = t;
      lastChatTs = now;
      return true;
    };

    const runGeneralChat = async (text) => {
      if (chatBusy) return;
      chatBusy = true;
      try {
        setStatus('대화 중...');
        const res = await fetch('/chat', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({ text }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) {
          const errMsg = (data && data.error) ? data.error : 'chat failed';
          setStatus(`대화 실패: ${errMsg}`);
          return;
        }
        const reply = (data.reply || '').trim();
        if (reply) {
          setStatus(reply);
          speakText(reply);
          if (voiceWanted) keepCompanionFollow(10000);
        } else {
          setStatus('대화 응답 없음');
        }
      } catch (e) {
        setStatus(`대화 실패: ${e}`);
      } finally {
        chatBusy = false;
      }
    };

    const executeIntent = async (intent, args = {}) => {
      const speedBase = parseFloat(speedEl.value);
      const turnBase = parseFloat(turnEl.value);
      const speedScaleRaw = parseFloat(args.speed_scale);
      const speedScale = Number.isFinite(speedScaleRaw) ? Math.min(1.5, Math.max(0.2, speedScaleRaw)) : 1.0;
      const speed = speedBase * speedScale;

      if (intent === 'stop') {
        stopFollowMode();
        stopMove();
        await callAutoNavApi('/autonav/stop', false);
        return 'stop';
      }
      if (intent === 'move_forward') {
        stopFollowMode();
        startMove(speed, 0);
        return 'forward';
      }
      if (intent === 'move_backward') {
        stopFollowMode();
        startMove(-speed, 0);
        return 'back';
      }
      if (intent === 'turn_left') {
        stopFollowMode();
        startMove(0, turnBase);
        return 'left';
      }
      if (intent === 'turn_right') {
        stopFollowMode();
        startMove(0, -turnBase);
        return 'right';
      }
      if (intent === 'follow_me') {
        stopFollowMode();
        stopMove();
        await callAutoNavApi('/autonav/start?mode=person_follow', false);
        return 'follow';
      }
      if (intent === 'auto_nav_start') {
        stopFollowMode();
        stopMove();
        await callAutoNavApi('/autonav/start?mode=person_follow', false);
        return 'autonav-start';
      }
      if (intent === 'auto_nav_stop') {
        await callAutoNavApi('/autonav/stop', false);
        return 'autonav-stop';
      }
      if (intent === 'save_home') {
        await callNavApi('/nav/home/set', false);
        return 'home-set';
      }
      if (intent === 'return_home') {
        stopFollowMode();
        stopMove();
        await callNavApi('/nav/return/start', false);
        return 'return-home';
      }
      if (intent === 'cancel_return') {
        await callNavApi('/nav/return/stop', false);
        return 'return-stop';
      }
      if (intent === 'park_reverse') {
        setStatus('후진 주차 기능은 준비 중입니다.');
        return 'park-reverse';
      }
      if (intent === 'park_slot') {
        let slot = parseInt(args.slot, 10);
        if (!Number.isFinite(slot)) slot = 5;
        slot = Math.max(1, Math.min(10, slot));
        if (phase2SlotEl) phase2SlotEl.value = String(slot);
        stopFollowMode();
        stopMove();
        await callDigitApi('/digit/stop');
        await callNumberApi('/number/stop');
        if (autoNavActive) await callAutoNavApi('/autonav/stop', false);
        await callAutoNavApi(`/autonav/start?mode=parking_phase2&slot=${slot}`, false);
        return `park-slot-${slot}`;
      }
      if (intent === 'describe_scene' || intent === 'chat') {
        return intent;
      }
      return 'unknown';
    };

    const runAssistantIntent = async (text) => {
      try {
        const res = await fetch('/assistant', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            text,
            recognized_name: latestRecognizedName || '',
          }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) return null;
        const intent = (data.intent || 'unknown').toString();
        const reply = (data.reply || '').toString().trim();
        const execute = !!data.execute;
        if (reply) {
          setStatus(reply);
          speakText(reply);
          if (intent === 'chat' && voiceWanted) keepCompanionFollow(10000);
        }
        if (!execute) return intent;
        const executed = await executeIntent(intent, data.args || {});
        return executed || intent;
      } catch (e) {
        return null;
      }
    };

    const setupRecognizer = () => {
      if (!SpeechRecognition) {
        setVoiceStatus('Speech API not supported');
        voiceBtn.disabled = true;
        return;
      }
      recognizer = new SpeechRecognition();
      recognizer.lang = 'ko-KR';
      recognizer.continuous = true;
      // Use interim results for faster command reaction.
      recognizer.interimResults = true;

      recognizer.onstart = () => {
        voiceActive = true;
        voiceBtn.classList.add('active');
        voiceBtn.textContent = 'Voice Stop';
        setVoiceStatus('Listening...');
        // First greeting at the beginning of each voice conversation session.
        if (latestRecognizedName && greetedNameInVoiceSession !== latestRecognizedName) {
          speakGreetingForName(latestRecognizedName);
          greetedNameInVoiceSession = latestRecognizedName;
        }
      };
      recognizer.onend = () => {
        voiceActive = false;
        if (voiceWanted) {
          setVoiceStatus('Reconnecting...');
          setTimeout(() => {
            if (!voiceActive && voiceWanted) {
              try {
                recognizer.start();
              } catch (e) {
                setVoiceStatus(`Voice restart failed: ${e}`);
              }
            }
          }, 200);
          return;
        }
        voiceBtn.classList.remove('active');
        voiceBtn.textContent = 'Voice Start';
        setVoiceStatus('Voice idle');
      };
      recognizer.onerror = (e) => {
        setVoiceStatus(`Voice error: ${e.error}`);
      };
      recognizer.onresult = async (e) => {
        const res = e.results[e.results.length - 1];
        if (!res || !res[0]) return;
        const isFinal = !!res.isFinal;
        const text = res[0].transcript || '';
        setVoiceLast(text);
        const spokenName = extractNameFromSpeech(text);
        if (spokenName && isFinal) {
          voiceCooldown = true;
          await registerFaceFromVoice(spokenName);
          setTimeout(() => { voiceCooldown = false; }, 600);
          return;
        }
        const wakeCommandText = extractWakeCommandText(text);
        if (wakeCommandText === null) {
          // Non-wake sentences are treated as normal conversation context.
          if (!isFinal) return;
          if (isSceneQuery(text)) {
            const summary = buildSceneSummary();
            setStatus(summary);
            speakText(summary);
            return;
          }
          if (!shouldRunChat(text)) return;
          await runGeneralChat(text);
          return;
        }
        if (voiceCooldown) return;
        if (!isFinal) return;

        // GPT-first intent routing for wake-word commands.
        const gptCmd = await runAssistantIntent(wakeCommandText);
        if (gptCmd && gptCmd !== 'unknown') {
          if (shouldRunVoiceCmd(gptCmd)) {
            setStatus(`Voice: ${gptCmd}`);
          }
          voiceCooldown = true;
          setTimeout(() => { voiceCooldown = false; }, 180);
          return;
        }

        // Fallback to local deterministic parser when GPT intent path is unavailable.
        const cmd = applyVoiceCommand(wakeCommandText);
        if (cmd === 'follow-active') return;
        if (cmd !== 'unknown' && shouldRunVoiceCmd(cmd)) {
          setStatus(`Voice: ${cmd}`);
          voiceCooldown = true;
          setTimeout(() => { voiceCooldown = false; }, 120);
          return;
        }
        setStatus('명령을 이해하지 못했어요. 예: 핑키 정지');
      };
    };

    setupRecognizer();

    voiceBtn.addEventListener('click', () => {
      if (!recognizer) return;
      if (!voiceWanted) {
        voiceWanted = true;
        greetedNameInVoiceSession = '';
        try {
          recognizer.start();
        } catch (e) {
          setVoiceStatus(`Voice start failed: ${e}`);
        }
      } else {
        voiceWanted = false;
        recognizer.stop();
      }
    });

    autoNavToggleBtn.addEventListener('click', async () => {
      if (autoNavActive) {
        await callAutoNavApi('/autonav/stop');
      } else {
        stopFollowMode();
        stopMove();
        await callDigitApi('/digit/stop');
        await callNumberApi('/number/stop');
        await callAutoNavApi('/autonav/start?mode=person_follow');
      }
    });

    phase1StartBtn.addEventListener('click', async () => {
      stopFollowMode();
      stopMove();
      await callDigitApi('/digit/stop');
      await callNumberApi('/number/stop');
      if (autoNavActive) await callAutoNavApi('/autonav/stop');
      await callAutoNavApi('/autonav/start?mode=orange_lane_phase1');
    });

    phase1StopBtn.addEventListener('click', async () => {
      if (autoNavActive && autoNavMode === 'orange_lane_phase1') {
        await callAutoNavApi('/autonav/stop');
      } else {
        setStatus('Phase1 모드가 실행 중이 아닙니다.');
      }
    });

    phase2StartBtn.addEventListener('click', async () => {
      stopFollowMode();
      stopMove();
      await callDigitApi('/digit/stop');
      await callNumberApi('/number/stop');
      if (autoNavActive) await callAutoNavApi('/autonav/stop');
      let slot = parseInt((phase2SlotEl && phase2SlotEl.value) ? phase2SlotEl.value : '5', 10);
      if (!Number.isFinite(slot)) slot = 5;
      slot = Math.max(1, Math.min(10, slot));
      if (phase2SlotEl) phase2SlotEl.value = String(slot);
      await callAutoNavApi(`/autonav/start?mode=parking_phase2&slot=${slot}`);
    });

    phase2StopBtn.addEventListener('click', async () => {
      if (autoNavActive && autoNavMode === 'parking_phase2') {
        await callAutoNavApi('/autonav/stop');
      } else {
        setStatus('Phase2 모드가 실행 중이 아닙니다.');
      }
    });

    phase2ResetStartBtn.addEventListener('click', async () => {
      stopFollowMode();
      stopMove();
      await callDigitApi('/digit/stop');
      await callNumberApi('/number/stop');
      if (autoNavActive) await callAutoNavApi('/autonav/stop');
      let slot = parseInt((phase2SlotEl && phase2SlotEl.value) ? phase2SlotEl.value : '5', 10);
      if (!Number.isFinite(slot)) slot = 5;
      slot = Math.max(1, Math.min(10, slot));
      if (phase2SlotEl) phase2SlotEl.value = String(slot);
      await callAutoNavApi(`/autonav/start?mode=parking_phase2&slot=${slot}&reset_start=1&start_cell=5,0&start_heading=1`);
    });

    const normalizePhase2Slot = () => {
      let slot = parseInt((phase2SlotEl && phase2SlotEl.value) ? phase2SlotEl.value : '5', 10);
      if (!Number.isFinite(slot)) slot = 5;
      slot = Math.max(1, Math.min(10, slot));
      if (phase2SlotEl) phase2SlotEl.value = String(slot);
      return slot;
    };
    phase2SlotEl.addEventListener('change', () => { normalizePhase2Slot(); });
    phase2SlotDecBtn.addEventListener('click', () => {
      const slot = normalizePhase2Slot();
      if (phase2SlotEl) phase2SlotEl.value = String(Math.max(1, slot - 1));
    });
    phase2SlotIncBtn.addEventListener('click', () => {
      const slot = normalizePhase2Slot();
      if (phase2SlotEl) phase2SlotEl.value = String(Math.min(10, slot + 1));
    });

    const callDigitApi = async (path) => {
      try {
        const res = await fetch(path);
        const text = (await res.text()).trim();
        if (!res.ok) {
          setStatus(text || `Digit API error: HTTP ${res.status}`);
          return false;
        }
        if (text) setStatus(text);
        return true;
      } catch (e) {
        setStatus(`Digit API error: ${e}`);
        return false;
      }
    };

    const startDigitById = async (markerId) => {
      stopFollowMode();
      stopMove();
      if (autoNavActive) {
        await callAutoNavApi('/autonav/stop');
      }
      const ok = await callDigitApi(`/digit/start?id=${markerId}`);
      if (ok) {
        setDigitStatus(`Digit: target #${markerId} starting`);
      }
    };

    digitGo1Btn.addEventListener('click', async () => {
      await startDigitById(1);
    });
    digitGo2Btn.addEventListener('click', async () => {
      await startDigitById(2);
    });
    digitGoIdBtn.addEventListener('click', async () => {
      const raw = (digitIdEl.value || '').trim();
      const v = parseInt(raw, 10);
      if (!Number.isFinite(v) || v < 0 || v > 49) {
        setStatus('Digit ID는 0~49여야 합니다.');
        return;
      }
      await startDigitById(v);
    });
    digitStopBtn.addEventListener('click', async () => {
      await callDigitApi('/digit/stop');
      setDigitStatus('Digit: stopped');
      digitGo1Btn.classList.remove('active');
      digitGo2Btn.classList.remove('active');
      digitGoIdBtn.classList.remove('active');
      digitStopBtn.classList.remove('active');
    });

    const callNumberApi = async (path) => {
      try {
        const res = await fetch(path);
        const text = (await res.text()).trim();
        if (!res.ok) {
          setStatus(text || `Number API error: HTTP ${res.status}`);
          return false;
        }
        if (text) setStatus(text);
        return true;
      } catch (e) {
        setStatus(`Number API error: ${e}`);
        return false;
      }
    };

    const startNumberById = async (n) => {
      stopFollowMode();
      stopMove();
      if (autoNavActive) await callAutoNavApi('/autonav/stop');
      await callDigitApi('/digit/stop');
      const ok = await callNumberApi(`/number/start?id=${n}`);
      if (ok) setNumberStatus(`Number: target #${n} starting`);
    };

    numberGo1Btn.addEventListener('click', async () => { await startNumberById(1); });
    numberGo2Btn.addEventListener('click', async () => { await startNumberById(2); });
    numberGo3Btn.addEventListener('click', async () => { await startNumberById(3); });
    numberGo4Btn.addEventListener('click', async () => { await startNumberById(4); });
    numberStopBtn.addEventListener('click', async () => {
      await callNumberApi('/number/stop');
      setNumberStatus('Number: stopped');
      numberGo1Btn.classList.remove('active');
      numberGo2Btn.classList.remove('active');
      numberGo3Btn.classList.remove('active');
      numberGo4Btn.classList.remove('active');
      numberStopBtn.classList.remove('active');
    });

    const fetchBattery = async () => {
      try {
        const res = await fetch('/telemetry');
        const data = await res.json();
        latestTelemetry = data || latestTelemetry;
        if (data && data.battery !== null) {
          batteryEl.textContent = `${data.battery.toFixed(2)} V`;
        }
        if (data && data.us !== null) {
          usEl.textContent = `${data.us.toFixed(2)} m`;
          if (data.us < 0.35) {
            usEl.classList.add('warn');
          } else {
            usEl.classList.remove('warn');
          }
        }
        if (data && data.ir && data.ir.length) {
          const [l, c, r] = data.ir;
          irEl.textContent = `L:${l} C:${c} R:${r}`;
        }
        if (data) {
          const fmt = (v) => (typeof v === 'number' && Number.isFinite(v)) ? v.toFixed(2) : '--';
          const sf = data.scan_min;
          const sl = data.scan_left;
          const sr = data.scan_right;
          const sb = data.scan_rear;
          lidarMinEl.textContent = `${fmt(sf)} / ${fmt(sl)} / ${fmt(sr)} / ${fmt(sb)} m`;

          let lidarState = 'LIDAR SAFE';
          let warn = false;
          const stopFront = (typeof sf === 'number' && sf <= 0.07);
          const stopRear = (typeof sb === 'number' && sb <= 0.06);
          const warnFront = (typeof sf === 'number' && sf <= 0.12);
          const warnRear = (typeof sb === 'number' && sb <= 0.11);
          const warnSide = ((typeof sl === 'number' && sl <= 0.30) || (typeof sr === 'number' && sr <= 0.30));
          if (stopFront) {
            lidarState = 'FRONT BLOCKED';
            warn = true;
          } else if (stopRear) {
            lidarState = 'REAR BLOCKED';
            warn = true;
          } else if (warnFront || warnRear || warnSide) {
            lidarState = 'LIDAR WARN';
            warn = true;
          }
          lidarStateEl.textContent = lidarState;
          lidarMinEl.classList.toggle('warn', warn);
          lidarStateEl.classList.toggle('warn', warn);
        }
        if (data && data.fps !== null && data.fps !== undefined) {
          fpsLive.textContent = data.fps.toFixed(1);
        }
        if (data && data.autonav) {
          latestAutoNav = data.autonav;
          setAutoNavUi(!!data.autonav.enabled, data.autonav.mode || '');
        }
        if (data && data.digit) {
          const d = data.digit;
          const enabled = !!d.enabled;
          const targetId = Number.isInteger(d.target_id) ? d.target_id : -1;
          const msg = (d.last_msg || '').toString();
          const age = (typeof d.last_seen_age === 'number') ? d.last_seen_age : null;
          const ageText = (age === null) ? '--' : `${age.toFixed(1)}s`;
          const known = Array.isArray(d.known_ids) ? d.known_ids.join(',') : '';
          const ordered = Array.isArray(d.ordered_ids_lr) ? d.ordered_ids_lr.join('>') : '';
          setDigitStatus(`Digit: ${enabled ? 'ON' : 'OFF'} target=${targetId} known=[${known}] lr=[${ordered}] msg=${msg || '--'} seen=${ageText}`);
          digitGo1Btn.classList.toggle('active', enabled && targetId === 1);
          digitGo2Btn.classList.toggle('active', enabled && targetId === 2);
          digitGoIdBtn.classList.toggle('active', enabled && targetId !== 1 && targetId !== 2);
          digitStopBtn.classList.toggle('active', enabled);
        }
        if (data && data.number) {
          const n = data.number;
          const enabled = !!n.enabled;
          const targetId = Number.isInteger(n.target_id) ? n.target_id : -1;
          const state = (n.state || '').toString();
          const msg = (n.last_msg || '').toString();
          const ordered = Array.isArray(n.ordered_ids_lr) ? n.ordered_ids_lr.join('>') : '';
          setNumberStatus(`Number: ${enabled ? 'ON' : 'OFF'} target=${targetId} state=${state} lr=[${ordered}] msg=${msg || '--'}`);
          numberGo1Btn.classList.toggle('active', enabled && targetId === 1);
          numberGo2Btn.classList.toggle('active', enabled && targetId === 2);
          numberGo3Btn.classList.toggle('active', enabled && targetId === 3);
          numberGo4Btn.classList.toggle('active', enabled && targetId === 4);
          numberStopBtn.classList.toggle('active', enabled);
        }
        if (data && data.greeting) {
          const name = String(data.greeting).trim();
          if (name) {
            const g = `안녕하세요. ${name}님, 오늘 처음 뵙네요.`;
            setStatus(g);
            speakText(g);
          }
        }
        if (data && data.faces && data.faces.length) {
          let best = null;
          let bestFaceObj = null;
          for (const f of data.faces) {
            if (!f || !f.name || f.name === 'unknown') continue;
            const score = (typeof f.score === 'number') ? f.score : 0;
            if (!best || score > best.score) {
              best = { name: f.name, score };
              bestFaceObj = f;
            }
          }
          latestRecognizedName = best ? best.name : '';
          latestBestFace = (bestFaceObj && data.face_size) ? { box: bestFaceObj.box, size: data.face_size } : null;
        } else {
          latestRecognizedName = '';
          latestBestFace = null;
        }
        if (!autoNavActive && followActive && Date.now() > conversationFollowUntil) {
          stopFollowMode();
          stopMove();
        }
        drawOverlay(
          data ? data.pose : null,
          data ? data.objects : null,
          data ? data.obj_size : null,
          data ? data.faces : null,
          data ? data.face_size : null
        );
      } catch (e) {
        // ignore
      }
    };
    fetchBattery();
    // Faster telemetry polling makes face/pose overlays feel responsive on mobile.
    setInterval(fetchBattery, 250);

    const clearOverlay = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    };

    // Floor-geometry guide overlay (calibrated for 320x240 stream).
    const FLOOR_GUIDE = {
      useFloorGuide: true,
      useFloorMask: true,
      horizonTopPx: 60,         // ~infinite distance singular line
      roiTopPx: 80,             // ignore top 1/3 for floor computation
      bottomCenterRmm: 90,
      bottomEdgeRmm: 107,
      centerRmm: 300,
      bottomHalfWidthMm: 57.8,
      bottomWidthMm: 115.6,
      maskMinIntervalMs: 120,
      usNoFloorM: 0.08,         // hard guard only at very close range
      closeIter: 2,             // morphological closing iterations for glare-induced gaps
      holeFillMaxPx: 260,       // fill small inner holes only
      minBlobPx: 120,           // remove tiny noisy blobs
      hysteresisDilateIter: 1,  // keep previous mask if current almost overlaps
      nearWallStripPx: 18,      // keep only bottom strip near wall
      nearWallStripMinPx: 180,  // minimum valid pixels to show strip
      cutWeakScore: 22,         // weak column threshold for cutline confidence
      globalCutScoreMin: 12,    // minimum global boundary score
      weakColRatioForGlobalCut: 0.30,
      usWallLikelyM: 0.12,      // near-wall hint (not hard stop)
      nearWallTopRatio: 0.30,   // clamp only when mask rises too high in near-wall state
      cutlineHistoryLen: 3,     // temporal median history for cutline stability
      nearWallGlobalCutForceM: 0.11, // force stronger global cut only when very close
      usCenterAnchorHalfRatio: 0.22, // ultrasonic anchor affects center columns only
      usMinValidMm: 70,
      usMaxValidMm: 1800,
      camHfovDeg: 62.0,
      wallAngleEmaAlpha: 0.60,
      usWallConfirmMm: 400,       // ultrasonic must also indicate near wall (40cm)
      camWallConfirmMm: 320,      // camera center estimated wall distance threshold
      camCenterConfirmHalfRatio: 0.10, // center band width for camera wall confirmation
      wallDirMedianLen: 3,        // open-dir temporal median
      sideEnterDeg: 10.0,         // sector switch enter threshold
      sideExitDeg: 6.0,           // sector switch exit threshold
      camSideConfirmMm: 320,      // side-sector near wall threshold
      camFrontFinalMm: 300,       // final center confirmation threshold
      wallBandPx: 26,             // draw only narrow wall band near boundary
      greenTopMinColsRatio: 0.10, // require minimum valid green-top columns
      greenTopOffsetPx: 2,        // small offset below green top for wall row
      greenRgbDiffMin: 20,        // green dominance threshold in RGB
      greenRgbRMax: 220,          // suppress bright white reflections
      greenRgbBMax: 220,
      greenTopMaxRowRatio: 0.68,  // reject green-top candidates too low (floor reflections)
      greenTopNeighborNeed: 2,    // require local horizontal support for green-top
    };
    let floorMaskLastTs = 0;
    let floorMaskPrev = null;
    const cutlineHist = [];
    let wallNav = { ready: false, angleDeg: 0.0, rangeMm: 0.0, x: 0, y: 0 };
    let wallMaskCoverage = 0.0;
    const wallDirHist = [];
    let wallSectorState = 'center'; // left/right/center (wall side)

    const morphDilate = (src, W, H, roiY, iter = 1) => {
      let a = src;
      for (let it = 0; it < iter; it++) {
        const out = new Uint8Array(W * H);
        for (let y = roiY; y < H; y++) {
          const ym = Math.max(roiY, y - 1);
          const yp = Math.min(H - 1, y + 1);
          for (let x = 0; x < W; x++) {
            const xm = Math.max(0, x - 1);
            const xp = Math.min(W - 1, x + 1);
            let on = 0;
            for (let yy = ym; yy <= yp && !on; yy++) {
              const row = yy * W;
              for (let xx = xm; xx <= xp; xx++) {
                if (a[row + xx]) {
                  on = 1;
                  break;
                }
              }
            }
            out[y * W + x] = on;
          }
        }
        a = out;
      }
      return a;
    };

    const morphErode = (src, W, H, roiY, iter = 1) => {
      let a = src;
      for (let it = 0; it < iter; it++) {
        const out = new Uint8Array(W * H);
        for (let y = roiY; y < H; y++) {
          const ym = Math.max(roiY, y - 1);
          const yp = Math.min(H - 1, y + 1);
          for (let x = 0; x < W; x++) {
            const xm = Math.max(0, x - 1);
            const xp = Math.min(W - 1, x + 1);
            let on = 1;
            for (let yy = ym; yy <= yp && on; yy++) {
              const row = yy * W;
              for (let xx = xm; xx <= xp; xx++) {
                if (!a[row + xx]) {
                  on = 0;
                  break;
                }
              }
            }
            out[y * W + x] = on;
          }
        }
        a = out;
      }
      return a;
    };

    const fillSmallHoles = (mask, W, H, roiY, maxHolePx) => {
      const vis = new Uint8Array(W * H);
      const q = new Int32Array(W * H);
      for (let y = roiY; y < H; y++) {
        for (let x = 0; x < W; x++) {
          const s = y * W + x;
          if (vis[s] || mask[s]) continue;
          let qh = 0;
          let qt = 0;
          q[qt++] = s;
          vis[s] = 1;
          let area = 0;
          let touchesBorder = false;
          while (qh < qt) {
            const i = q[qh++];
            area++;
            const cy = (i / W) | 0;
            const cx = i - (cy * W);
            if (cx === 0 || cx === (W - 1) || cy === roiY || cy === (H - 1)) {
              touchesBorder = true;
            }
            if (cx > 0) {
              const ni = i - 1;
              if (!vis[ni] && !mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cx < (W - 1)) {
              const ni = i + 1;
              if (!vis[ni] && !mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cy > roiY) {
              const ni = i - W;
              if (!vis[ni] && !mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cy < (H - 1)) {
              const ni = i + W;
              if (!vis[ni] && !mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
          }
          if (!touchesBorder && area <= maxHolePx) {
            for (let k = 0; k < qt; k++) mask[q[k]] = 1;
          }
        }
      }
      return mask;
    };

    const removeSmallIslands = (mask, W, H, roiY, minBlobPx) => {
      const vis = new Uint8Array(W * H);
      const q = new Int32Array(W * H);
      for (let y = roiY; y < H; y++) {
        for (let x = 0; x < W; x++) {
          const s = y * W + x;
          if (vis[s] || !mask[s]) continue;
          let qh = 0;
          let qt = 0;
          q[qt++] = s;
          vis[s] = 1;
          while (qh < qt) {
            const i = q[qh++];
            const cy = (i / W) | 0;
            const cx = i - (cy * W);
            if (cx > 0) {
              const ni = i - 1;
              if (!vis[ni] && mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cx < (W - 1)) {
              const ni = i + 1;
              if (!vis[ni] && mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cy > roiY) {
              const ni = i - W;
              if (!vis[ni] && mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
            if (cy < (H - 1)) {
              const ni = i + W;
              if (!vis[ni] && mask[ni]) {
                vis[ni] = 1;
                q[qt++] = ni;
              }
            }
          }
          if (qt < minBlobPx) {
            for (let k = 0; k < qt; k++) mask[q[k]] = 0;
          }
        }
      }
      return mask;
    };

    const drawMaskArray = (mask, W, H) => {
      const out = floorMaskCtx.createImageData(W, H);
      const o = out.data;
      let on = 0;
      for (let i = 0; i < (W * H); i++) {
        if (!mask[i]) continue;
        on += 1;
        const p = i * 4;
        // Green wall mask (high-visibility).
        o[p] = 20;
        o[p + 1] = 255;
        o[p + 2] = 110;
        o[p + 3] = 92;
      }
      wallMaskCoverage = (on / Math.max(1, W * H));
      floorMaskCtx.putImageData(out, 0, 0);
    };

    const solveCenterRangeModel = (H, roiY) => {
      const h = Math.max(0, Math.min(H - 3, FLOOR_GUIDE.horizonTopPx));
      const yb = H - 1;
      const yc = Math.max(roiY + 6, Math.round(H * 0.5));
      const rb = FLOOR_GUIDE.bottomCenterRmm;
      const rc = FLOOR_GUIDE.centerRmm;
      const db = yb - h;
      const dc = yc - h;
      if (db <= 1 || dc <= 1 || Math.abs(rc - rb) < 1) return null;
      const inv = (1.0 / dc) - (1.0 / db);
      if (Math.abs(inv) < 1e-6) return null;
      const K = (rc - rb) / inv;
      const C = rb - (K / db);
      return { h, K, C };
    };

    const centerRowToRangeMm = (y, H, roiY) => {
      const m = solveCenterRangeModel(H, roiY);
      if (!m) return null;
      const den = (Math.max(roiY + 1, y) - m.h);
      if (den <= 1e-3) return null;
      const r = (m.K / den) + m.C;
      if (!Number.isFinite(r)) return null;
      return r;
    };

    const medianOf = (arr) => {
      if (!arr || !arr.length) return null;
      const v = arr.slice().sort((a, b) => a - b);
      return v[(v.length / 2) | 0];
    };

    // Convert center-line floor distance(mm) to image row(y) using calibrated rational model.
    // R(y) ~= K/(y-h) + C, fitted by:
    // - bottom center (R=90mm at y=H-1)
    // - image center (R=300mm at y=H/2)
    const centerRangeMmToRow = (rangeMm, H, roiY) => {
      const m = solveCenterRangeModel(H, roiY);
      if (!m) return null;
      const denom = (rangeMm - m.C);
      if (denom <= 1e-3) return null;
      const y = m.h + (m.K / denom);
      if (!Number.isFinite(y)) return null;
      return Math.max(roiY, Math.min(H - 1, Math.round(y)));
    };

    const drawFloorGuide = () => {
      if (!FLOOR_GUIDE.useFloorGuide) return;
      const w = canvas.width;
      const h = canvas.height;
      if (w <= 0 || h <= 0) return;

      const sy = h / 240.0;
      const horizonY = Math.round(FLOOR_GUIDE.horizonTopPx * sy);
      const roiY = Math.round(FLOOR_GUIDE.roiTopPx * sy);
      const cx = w * 0.5;
      const cy = h * 0.5;
      const bottomY = h - 1;

      // Do not paint translucent ROI area fills.
      // Keep only guide lines/text to avoid confusion with the floor mask.

      // Singular line (distance diverges) and ROI start line.
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = 'rgba(255, 210, 0, 0.95)';
      ctx.beginPath();
      ctx.moveTo(0, horizonY + 0.5);
      ctx.lineTo(w, horizonY + 0.5);
      ctx.stroke();

      ctx.strokeStyle = 'rgba(60, 210, 255, 0.95)';
      ctx.beginPath();
      ctx.moveTo(0, roiY + 0.5);
      ctx.lineTo(w, roiY + 0.5);
      ctx.stroke();

      // Center reference ray and calibration points.
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.55)';
      ctx.beginPath();
      ctx.moveTo(cx + 0.5, roiY);
      ctx.lineTo(cx + 0.5, bottomY);
      ctx.stroke();

      ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
      ctx.beginPath();
      ctx.arc(cx, bottomY, 3, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.arc(cx, cy, 3, 0, Math.PI * 2);
      ctx.fill();

      // Bottom calibration span (R=107mm at left/right edge, R=90mm at center).
      const mmPerPxBottom = FLOOR_GUIDE.bottomWidthMm / Math.max(1.0, w);
      const dx = FLOOR_GUIDE.bottomHalfWidthMm / mmPerPxBottom;
      const lx = Math.max(0, Math.round(cx - dx));
      const rx = Math.min(w - 1, Math.round(cx + dx));
      ctx.strokeStyle = 'rgba(0, 255, 160, 0.95)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(lx, bottomY - 0.5);
      ctx.lineTo(rx, bottomY - 0.5);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(lx, bottomY, 2.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.arc(rx, bottomY, 2.5, 0, Math.PI * 2);
      ctx.fill();

      ctx.font = '11px sans-serif';
      ctx.textBaseline = 'top';
      ctx.fillStyle = 'rgba(255, 220, 120, 0.95)';
      ctx.fillText('Floor singular line (~inf)', 6, Math.max(0, horizonY - 14));
      ctx.fillStyle = 'rgba(100, 220, 255, 0.95)';
      ctx.fillText('Floor ROI start (top 1/3 excluded)', 6, roiY + 4);
      ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
      ctx.fillText(`C(bottom): ${FLOOR_GUIDE.bottomCenterRmm}mm`, Math.round(cx) + 6, Math.max(roiY + 18, bottomY - 36));
      ctx.fillText(`C(center): ${FLOOR_GUIDE.centerRmm}mm`, Math.round(cx) + 6, Math.max(roiY + 34, Math.round(cy) - 8));
      ctx.fillStyle = 'rgba(0, 255, 160, 0.95)';
      ctx.fillText(`Edge: ${FLOOR_GUIDE.bottomEdgeRmm}mm`, Math.max(6, lx + 4), Math.max(roiY + 50, bottomY - 20));
    };

    const updateFloorMask = () => {
      if (!FLOOR_GUIDE.useFloorMask) return;
      const us = latestTelemetry && (typeof latestTelemetry.us === 'number') ? latestTelemetry.us : null;
      const now = Date.now();
      if ((now - floorMaskLastTs) < FLOOR_GUIDE.maskMinIntervalMs) return;
      if (!imgEl.complete || !imgEl.naturalWidth || !imgEl.naturalHeight) return;
      floorMaskLastTs = now;

      const W = floorAnalyzeCanvas.width;
      const H = floorAnalyzeCanvas.height;
      const n = W * H;
      const roiY = Math.max(0, Math.min(H - 1, FLOOR_GUIDE.roiTopPx));
      let usAnchorY = null;
      let usPriorY = null;
      let usPriorTrust = false;

      // Build a prior wall row from ultrasonic when value is plausible.
      // This prevents floor markings (e.g., orange stripe) from being picked as wall boundary.
      if (us !== null) {
        const usMm = us * 1000.0;
        if (usMm >= FLOOR_GUIDE.usMinValidMm && usMm <= FLOOR_GUIDE.usMaxValidMm) {
          const yUs = centerRangeMmToRow(usMm, H, roiY);
          if (yUs !== null) {
            usPriorY = yUs;
            // Far/medium distance prior is reliable for rejecting near-bottom false edges.
            usPriorTrust = (usMm >= 250.0);
          }
        }
      }

      floorAnalyzeCtx.drawImage(imgEl, 0, 0, W, H);
      const src = floorAnalyzeCtx.getImageData(0, 0, W, H);
      const data = src.data;
      const cand = new Uint8Array(n);
      const reach = new Uint8Array(n);
      const lum = new Uint8Array(n);
      const satMap = new Uint8Array(n);
      const upBarrier = new Uint8Array(n); // blocks propagation from (x,y) to (x,y-1)
      const cutline = new Int16Array(W);   // per-column floor/wall split line
      const weakCol = new Uint8Array(W);
      const colScore = new Float32Array(W);
      const greenTop = new Int16Array(W);
      for (let x = 0; x < W; x++) greenTop[x] = -1;
      const qx = new Int16Array(n);
      const qy = new Int16Array(n);
      let qh = 0;
      let qt = 0;

      // Rule-based floor candidate: supports dark + ivory tiles in ROI.
      for (let y = roiY; y < H; y++) {
        let idx = y * W;
        let p = idx * 4;
        for (let x = 0; x < W; x++, idx++, p += 4) {
          const r = data[p];
          const g = data[p + 1];
          const b = data[p + 2];
          const cmax = Math.max(r, g, b);
          const cmin = Math.min(r, g, b);
          const diff = cmax - cmin;
          const s = cmax === 0 ? 0 : (255 * diff) / cmax;
          const v = cmax;
          const y8 = (77 * r + 150 * g + 29 * b) >> 8;
          lum[idx] = y8;
          satMap[idx] = s;
          const floorLike =
            ((s <= 120) && (v >= 22) && (v <= 250)) ||
            ((s <= 70) && (v >= 155));
          cand[idx] = floorLike ? 1 : 0;
          if (greenTop[x] < 0) {
            const greenLike =
              (g >= 55) &&
              (g > r + FLOOR_GUIDE.greenRgbDiffMin) &&
              (g > b + FLOOR_GUIDE.greenRgbDiffMin) &&
              (r <= FLOOR_GUIDE.greenRgbRMax) &&
              (b <= FLOOR_GUIDE.greenRgbBMax);
            if (greenLike) greenTop[x] = y;
          }
        }
      }

      // Build vertical propagation barriers from strong row-to-row appearance changes.
      // This blocks floor fill from leaking across floor-wall boundaries when camera yaw changes.
      for (let y = roiY + 1; y < H; y++) {
        let idx = y * W;
        let prev = (y - 1) * W;
        for (let x = 0; x < W; x++, idx++, prev++) {
          const dLum = Math.abs(lum[idx] - lum[prev]);
          const dSat = Math.abs(satMap[idx] - satMap[prev]);
          // Orange wall line + wall boundary usually creates a strong transition.
          if (dLum >= 26 || dSat >= 36) {
            upBarrier[idx] = 1;
          }
        }
      }

      // Estimate a slanted floor-wall boundary line per column.
      // This stabilizes masking when camera yaw changes.
      let weakCount = 0;
      for (let x = 0; x < W; x++) {
        let bestY = roiY;
        let bestScore = 0;
        let y0 = Math.max(roiY + 4, 8);
        let y1 = H - 8;
        if (usPriorTrust && usPriorY !== null) {
          // Keep search around ultrasonic-expected wall row.
          const bandTop = Math.max(y0, usPriorY - 42);
          const bandBot = Math.min(y1, usPriorY + 84);
          if (bandBot > bandTop + 6) {
            y0 = bandTop;
            y1 = bandBot;
          }
        }
        for (let y = y0; y < y1; y++) {
          const i = y * W + x;
          const p = (y - 1) * W + x;
          const dLum = Math.abs(lum[i] - lum[p]);
          const dSat = Math.abs(satMap[i] - satMap[p]);
          let score = (0.72 * dLum) + (0.28 * dSat);
          if (usPriorTrust && usPriorY !== null) {
            const dy = Math.abs(y - usPriorY);
            // Strongly penalize far-from-prior boundaries (typical floor stripe false-positive).
            score -= (0.85 * dy);
          }
          if (score > bestScore) {
            bestScore = score;
            bestY = y;
          }
        }
        // If boundary evidence is weak, fallback to ROI top.
        if (bestScore >= FLOOR_GUIDE.cutWeakScore) {
          cutline[x] = bestY;
          weakCol[x] = 0;
          colScore[x] = bestScore;
        } else {
          cutline[x] = roiY;
          weakCol[x] = 1;
          colScore[x] = bestScore;
          weakCount++;
        }
      }

      // Fill weak columns from nearest strong neighbors to avoid local leak (e.g., right side).
      if (weakCount > 0 && weakCount < W) {
        const leftStrong = new Int16Array(W);
        const rightStrong = new Int16Array(W);
        let last = -1;
        for (let x = 0; x < W; x++) {
          if (!weakCol[x]) last = x;
          leftStrong[x] = last;
        }
        last = -1;
        for (let x = W - 1; x >= 0; x--) {
          if (!weakCol[x]) last = x;
          rightStrong[x] = last;
        }
        for (let x = 0; x < W; x++) {
          if (!weakCol[x]) continue;
          const lx = leftStrong[x];
          const rx = rightStrong[x];
          if (lx >= 0 && rx >= 0 && lx !== rx) {
            const t = (x - lx) / Math.max(1, (rx - lx));
            const y = ((1.0 - t) * cutline[lx]) + (t * cutline[rx]);
            cutline[x] = Math.round(y);
          } else if (lx >= 0) {
            cutline[x] = cutline[lx];
          } else if (rx >= 0) {
            cutline[x] = cutline[rx];
          }
        }
      }

      // Smooth cutline to reduce jitter/frame flicker.
      const smooth = new Int16Array(W);
      for (let x = 0; x < W; x++) {
        let acc = 0;
        let cnt = 0;
        for (let k = -4; k <= 4; k++) {
          const xx = x + k;
          if (xx < 0 || xx >= W) continue;
          acc += cutline[xx];
          cnt += 1;
        }
        smooth[x] = Math.round(acc / Math.max(1, cnt));
      }
      for (let x = 0; x < W; x++) cutline[x] = smooth[x];

      // Reject unstable green-top candidates from floor reflections.
      // Valid wall top should appear in upper-mid image and have horizontal support.
      {
        const yMax = Math.round(H * FLOOR_GUIDE.greenTopMaxRowRatio);
        for (let x = 0; x < W; x++) {
          const gy = greenTop[x];
          if (gy < 0) continue;
          if (gy > yMax) {
            greenTop[x] = -1;
            continue;
          }
          let n = 0;
          for (let k = -2; k <= 2; k++) {
            if (k === 0) continue;
            const xx = x + k;
            if (xx < 0 || xx >= W) continue;
            if (greenTop[xx] >= 0 && Math.abs(greenTop[xx] - gy) <= 14) n += 1;
          }
          if (n < FLOOR_GUIDE.greenTopNeighborNeed) {
            greenTop[x] = -1;
          }
        }
      }

      // Wall row must prefer the top edge of green wall paint when available.
      // This rejects floor-tile boundaries falsely chosen as wall boundary.
      let greenCols = 0;
      for (let x = 0; x < W; x++) {
        if (greenTop[x] >= 0) greenCols += 1;
      }
      if ((greenCols / Math.max(1, W)) >= FLOOR_GUIDE.greenTopMinColsRatio) {
        for (let x = 0; x < W; x++) {
          if (greenTop[x] < 0) continue;
          const yTop = Math.max(roiY, Math.min(H - 1, greenTop[x] + FLOOR_GUIDE.greenTopOffsetPx));
          cutline[x] = Math.min(cutline[x], yTop);
        }
      }

      // If many columns are weak (e.g., glare/white stripe), derive one global boundary row.
      let globalCutY = roiY;
      let globalCutScore = 0;
      if ((weakCount / Math.max(1, W)) >= FLOOR_GUIDE.weakColRatioForGlobalCut) {
        let gBestY = roiY;
        let gBest = 0;
        for (let y = roiY + 4; y < H - 6; y++) {
          let acc = 0;
          const row = y * W;
          const prev = (y - 1) * W;
          for (let x = 0; x < W; x++) {
            const dLum = Math.abs(lum[row + x] - lum[prev + x]);
            const dSat = Math.abs(satMap[row + x] - satMap[prev + x]);
            acc += (0.72 * dLum) + (0.28 * dSat);
          }
          const avg = acc / Math.max(1, W);
          if (avg > gBest) {
            gBest = avg;
            gBestY = y;
          }
        }
        globalCutY = gBestY;
        globalCutScore = gBest;
        if (gBest >= FLOOR_GUIDE.globalCutScoreMin) {
          for (let x = 0; x < W; x++) {
            if (weakCol[x]) cutline[x] = Math.max(cutline[x], gBestY);
          }
        }
      }

      // Temporal median on cutline to reduce single-frame break by glare/stripe disappearance.
      cutlineHist.push(new Int16Array(cutline));
      while (cutlineHist.length > FLOOR_GUIDE.cutlineHistoryLen) cutlineHist.shift();
      if (cutlineHist.length >= 3) {
        const t = cutlineHist.length;
        const tmp = new Int16Array(t);
        for (let x = 0; x < W; x++) {
          for (let k = 0; k < t; k++) tmp[k] = cutlineHist[k][x];
          for (let i = 0; i < t - 1; i++) {
            for (let j = i + 1; j < t; j++) {
              if (tmp[j] < tmp[i]) {
                const v = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = v;
              }
            }
          }
          cutline[x] = tmp[(t / 2) | 0];
        }
      }

      // Very close to wall: trust global boundary more than per-column fragile edges.
      if (us !== null && us <= FLOOR_GUIDE.nearWallGlobalCutForceM) {
        if (globalCutScore >= (FLOOR_GUIDE.globalCutScoreMin * 0.8)) {
          for (let x = 0; x < W; x++) {
            cutline[x] = Math.max(cutline[x], globalCutY);
          }
        }
      }

      // Ultrasonic center anchor:
      // use ultrasonic distance as "center ray wall row" hint, then clamp center cutline.
      if (us !== null) {
        const usMm = us * 1000.0;
        if (usMm >= FLOOR_GUIDE.usMinValidMm && usMm <= FLOOR_GUIDE.usMaxValidMm) {
          const yUs = centerRangeMmToRow(usMm, H, roiY);
          if (yUs !== null) {
            usAnchorY = Math.max(roiY + 2, Math.min(H - 2, yUs));
            const cx = Math.round((W - 1) * 0.5);
            const half = Math.max(10, Math.round(W * FLOOR_GUIDE.usCenterAnchorHalfRatio));
            const x0 = Math.max(0, cx - half);
            const x1 = Math.min(W - 1, cx + half);
            for (let x = x0; x <= x1; x++) {
              cutline[x] = Math.max(cutline[x], usAnchorY);
            }
          }
        }
      }

      // Wall-first heading: choose direction where wall is farthest (smallest cutline y).
      let rawOpenDeg = null;
      {
        const hfov = FLOOR_GUIDE.camHfovDeg * (Math.PI / 180.0);
        const cx = (W - 1) * 0.5;
        const xMargin = Math.round(W * 0.08);
        let bestX = -1;
        let bestY = H;
        let bestScore = -1e9;
        for (let x = xMargin; x < (W - xMargin); x++) {
          const y = cutline[x];
          if (y <= roiY || y >= (H - 2)) continue;
          const conf = Math.max(0.0, Math.min(1.0, colScore[x] / Math.max(1.0, FLOOR_GUIDE.cutWeakScore)));
          const open = (H - y); // bigger is more floor ahead before wall
          const edgePenalty = Math.abs((x - cx) / Math.max(1.0, cx));
          const score = (open * (0.7 + 0.3 * conf)) - (10.0 * edgePenalty);
          if (score > bestScore) {
            bestScore = score;
            bestX = x;
            bestY = y;
          }
        }
        if (bestX >= 0) {
          const nx = (bestX - cx) / Math.max(1.0, cx);
          const ang = Math.atan(nx * Math.tan(hfov * 0.5)) * (180.0 / Math.PI);
          rawOpenDeg = ang;
          const rmm = centerRowToRangeMm(bestY, H, roiY);
          const aPrev = wallNav.ready ? wallNav.angleDeg : ang;
          const aNew = (FLOOR_GUIDE.wallAngleEmaAlpha * ang) + ((1.0 - FLOOR_GUIDE.wallAngleEmaAlpha) * aPrev);
          wallNav = {
            ready: true,
            angleDeg: aNew,
            rangeMm: (rmm && Number.isFinite(rmm)) ? rmm : 0.0,
            x: bestX,
            y: bestY,
          };
        }
      }

      // Conservative wall gate with sector-aware confirmation.
      let wallConfirmed = false;
      let wallX0 = 0;
      let wallX1 = W - 1;
      if (us !== null) {
        const usMm = us * 1000.0;
        const cx = Math.round((W - 1) * 0.5);
        if (rawOpenDeg !== null && Number.isFinite(rawOpenDeg)) {
          wallDirHist.push(rawOpenDeg);
          while (wallDirHist.length > FLOOR_GUIDE.wallDirMedianLen) wallDirHist.shift();
        }
        const openStable = medianOf(wallDirHist);
        if (openStable !== null) {
          // Open direction right(+): wall is likely on left, and vice versa.
          if (wallSectorState === 'center') {
            if (openStable >= FLOOR_GUIDE.sideEnterDeg) wallSectorState = 'left';
            else if (openStable <= -FLOOR_GUIDE.sideEnterDeg) wallSectorState = 'right';
          } else if (wallSectorState === 'left') {
            if (openStable < FLOOR_GUIDE.sideExitDeg) wallSectorState = 'center';
          } else if (wallSectorState === 'right') {
            if (openStable > -FLOOR_GUIDE.sideExitDeg) wallSectorState = 'center';
          }
        }

        const bandMm = (x0, x1) => {
          const ys = [];
          for (let x = Math.max(0, x0); x <= Math.min(W - 1, x1); x++) {
            const y = cutline[x];
            if (y > roiY && y < H) ys.push(y);
          }
          if (!ys.length) return null;
          const yMed = medianOf(ys);
          return centerRowToRangeMm(yMed, H, roiY);
        };
        const cHalf = Math.max(6, Math.round(W * FLOOR_GUIDE.camCenterConfirmHalfRatio));
        const camCenterMm = bandMm(cx - cHalf, cx + cHalf);
        let camSideMm = camCenterMm;
        if (wallSectorState === 'left') {
          camSideMm = bandMm(Math.round(W * 0.05), Math.round(W * 0.35));
        } else if (wallSectorState === 'right') {
          camSideMm = bandMm(Math.round(W * 0.65), Math.round(W * 0.95));
        }
        const usNear = Number.isFinite(usMm) && (usMm <= FLOOR_GUIDE.usWallConfirmMm);
        const camSideNear = (camSideMm !== null) && Number.isFinite(camSideMm) && (camSideMm <= FLOOR_GUIDE.camSideConfirmMm);
        const camFrontNear = (camCenterMm !== null) && Number.isFinite(camCenterMm) && (camCenterMm <= FLOOR_GUIDE.camFrontFinalMm);
        // Side wall: confirm by side sector only. Front wall: require center near.
        if (wallSectorState === 'left') {
          wallConfirmed = (usNear && camSideNear);
          wallX0 = 0;
          wallX1 = Math.round(W * 0.45);
        } else if (wallSectorState === 'right') {
          wallConfirmed = (usNear && camSideNear);
          wallX0 = Math.round(W * 0.55);
          wallX1 = W - 1;
        } else {
          wallConfirmed = (usNear && camFrontNear);
          wallX0 = 0;
          wallX1 = W - 1;
        }
      }

      // Wall-only mask:
      // 1) If green wall top exists, paint from ROI top down to that green top (user requirement).
      // 2) Else fallback to cutline-based band.
      let wallMask = new Uint8Array(n);
      let greenTopPainted = false;
      if ((greenCols / Math.max(1, W)) >= FLOOR_GUIDE.greenTopMinColsRatio) {
        for (let x = 0; x < W; x++) {
          if (greenTop[x] < 0) continue;
          const yGreen = Math.max(roiY, Math.min(H - 1, greenTop[x] + FLOOR_GUIDE.greenTopOffsetPx));
          for (let y = roiY; y <= yGreen; y++) {
            wallMask[(y * W) + x] = 1;
          }
        }
        greenTopPainted = true;
      }
      if (wallConfirmed && !greenTopPainted) {
        for (let x = Math.max(0, wallX0); x <= Math.min(W - 1, wallX1); x++) {
          const yWall = Math.max(roiY, Math.min(H - 1, cutline[x]));
          const yTop = Math.max(roiY, yWall - FLOOR_GUIDE.wallBandPx);
          for (let y = yTop; y < yWall; y++) {
            wallMask[(y * W) + x] = 1;
          }
        }
        wallMask = morphDilate(wallMask, W, H, roiY, 1);
        wallMask = morphErode(wallMask, W, H, roiY, 1);
      } else if (!greenTopPainted) {
        // No dual-confirmation => do not render wall mask.
        wallNav.ready = false;
      }

      // Temporal hysteresis on wall mask for flicker reduction.
      if (floorMaskPrev && floorMaskPrev.length === n) {
        const curDil = morphDilate(wallMask, W, H, roiY, FLOOR_GUIDE.hysteresisDilateIter);
        for (let i = 0; i < n; i++) {
          if (!wallMask[i] && floorMaskPrev[i] && curDil[i]) {
            wallMask[i] = 1;
          }
        }
      }

      floorMaskPrev = wallMask;
      drawMaskArray(wallMask, W, H);
    };

    const drawWallNav = () => {
      if (!wallNav || !wallNav.ready) {
        if (wallMaskCoverage > 0.0001) {
          ctx.font = '12px sans-serif';
          ctx.textBaseline = 'top';
          ctx.fillStyle = 'rgba(80, 255, 140, 0.95)';
          ctx.fillText(`Green wall mask: ${(wallMaskCoverage * 100.0).toFixed(1)}%`, 6, 6);
        }
        return;
      }
      const w = canvas.width;
      const h = canvas.height;
      if (w <= 0 || h <= 0) return;
      const sx = w / floorAnalyzeCanvas.width;
      const sy = h / floorAnalyzeCanvas.height;
      const x = wallNav.x * sx;
      const y = wallNav.y * sy;
      ctx.strokeStyle = 'rgba(255, 120, 40, 0.95)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x, y - 10);
      ctx.lineTo(x, y + 10);
      ctx.stroke();
      ctx.font = '12px sans-serif';
      ctx.textBaseline = 'top';
      ctx.fillStyle = 'rgba(255, 180, 80, 0.98)';
      const dTxt = wallNav.rangeMm > 0 ? `${Math.round(wallNav.rangeMm)}mm` : '--';
      ctx.fillText(`Wall open dir: ${wallNav.angleDeg.toFixed(1)}deg  far:${dTxt}`, 6, Math.max(0, y - 18));
      ctx.fillStyle = 'rgba(80, 255, 140, 0.95)';
      ctx.fillText(`Green wall mask: ${(wallMaskCoverage * 100.0).toFixed(1)}%`, 6, Math.max(0, y - 2));
    };

    const drawFloorMask = () => {
      if (!FLOOR_GUIDE.useFloorMask) return;
      updateFloorMask();
      const w = canvas.width;
      const h = canvas.height;
      if (w <= 0 || h <= 0) return;
      ctx.drawImage(floorMaskCanvas, 0, 0, w, h);
    };

    const drawPhase2Keepout = () => {};

    const drawPose = (pose) => {
      if (!pose || !pose.w || !pose.h) return;
      const sx = canvas.width / pose.w;
      const sy = canvas.height / pose.h;

      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(0, 255, 255, 0.9)';
      ctx.fillStyle = 'rgba(255, 200, 0, 0.9)';

      if (pose.boxes && pose.boxes.length) {
        pose.boxes.forEach((b) => {
          const x1 = b[0] * sx;
          const y1 = b[1] * sy;
          const x2 = b[2] * sx;
          const y2 = b[3] * sy;
          ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        });
      }

      if (pose.kps && pose.kps.length) {
        pose.kps.forEach((person) => {
          person.forEach((p) => {
            const x = p[0] * sx;
            const y = p[1] * sy;
            ctx.beginPath();
            ctx.arc(x, y, 3, 0, Math.PI * 2);
            ctx.fill();
          });
        });
      }
    };

    const drawFaces = (faces, size) => {
      if (!faces || !faces.length) return;
      let sx = 1;
      let sy = 1;
      if (size && size.w && size.h) {
        sx = canvas.width / size.w;
        sy = canvas.height / size.h;
      }
      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(0, 255, 0, 0.85)';
      ctx.fillStyle = 'rgba(0, 255, 0, 0.85)';
      ctx.font = '12px sans-serif';
      ctx.textBaseline = 'bottom';
      faces.forEach((f) => {
        if (!f || !f.box) return;
        const x1 = f.box[0] * sx;
        const y1 = f.box[1] * sy;
        const x2 = f.box[2] * sx;
        const y2 = f.box[3] * sy;
        ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        const label = f.name ? `${f.name}${f.score ? ` ${(f.score * 100).toFixed(0)}%` : ''}` : 'unknown';
        ctx.fillText(label, x1 + 4, y1 - 4);
      });
    };

    const drawObjects = (objects, size) => {
      if (!objects || !objects.length) return;
      let sx = 1;
      let sy = 1;
      if (size && size.w && size.h) {
        sx = canvas.width / size.w;
        sy = canvas.height / size.h;
      }
      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(255, 140, 0, 0.9)';
      ctx.fillStyle = 'rgba(255, 180, 70, 0.95)';
      ctx.font = '12px sans-serif';
      ctx.textBaseline = 'bottom';
      objects.forEach((o) => {
        if (!o || !o.box || !o.label) return;
        const x1 = o.box[0] * sx;
        const y1 = o.box[1] * sy;
        const x2 = o.box[2] * sx;
        const y2 = o.box[3] * sy;
        ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        const score = (typeof o.score === 'number') ? ` ${(o.score * 100).toFixed(0)}%` : '';
        ctx.fillText(`${o.label}${score}`, x1 + 4, y1 - 4);
      });
    };

    const drawPhase2ParkingMask = () => {
      if (!(autoNavActive && autoNavMode === 'parking_phase2')) return;
      const a = latestAutoNav;
      if (!a) return;
      const bb = a.phase2_parking_bbox;
      if (!Array.isArray(bb) || bb.length < 4) return;
      const x = Number(bb[0]);
      const y = Number(bb[1]);
      const w = Number(bb[2]);
      const h = Number(bb[3]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(w) || !Number.isFinite(h)) return;
      if (w < 2 || h < 2) return;

      const sx = canvas.width / 320.0;
      const sy = canvas.height / 240.0;
      const rx = x * sx;
      const ry = y * sy;
      const rw = w * sx;
      const rh = h * sy;
      const conf = Number(a.phase2_parking_conf || 0);

      ctx.fillStyle = 'rgba(0, 120, 255, 0.22)';
      ctx.strokeStyle = 'rgba(0, 180, 255, 0.98)';
      ctx.lineWidth = 2;
      ctx.fillRect(rx, ry, rw, rh);
      ctx.strokeRect(rx + 0.5, ry + 0.5, Math.max(1, rw - 1), Math.max(1, rh - 1));
      ctx.font = '12px sans-serif';
      ctx.textBaseline = 'top';
      ctx.fillStyle = 'rgba(170, 230, 255, 0.98)';
      ctx.fillText(`Parking area conf=${conf.toFixed(2)}`, Math.max(6, rx + 4), Math.max(0, ry - 16));

      // Draw the real detected blue Y-axis guide line from backend.
      const gl = a.phase2_guide_line;
      if (Array.isArray(gl) && gl.length >= 4) {
        const gx1 = Number(gl[0]) * sx;
        const gy1 = Number(gl[1]) * sy;
        const gx2 = Number(gl[2]) * sx;
        const gy2 = Number(gl[3]) * sy;
        if (Number.isFinite(gx1) && Number.isFinite(gy1) && Number.isFinite(gx2) && Number.isFinite(gy2)) {
          ctx.strokeStyle = 'rgba(0, 255, 255, 0.98)';
          ctx.lineWidth = 3;
          ctx.beginPath();
          ctx.moveTo(gx1, gy1);
          ctx.lineTo(gx2, gy2);
          ctx.stroke();
        }
      }
    };

    const drawOverlay = (pose, objects, objSize, faces, faceSize) => {
      clearOverlay();
      drawFloorMask();
      drawFloorGuide();
      drawPhase2Keepout();
      drawWallNav();
      drawPhase2ParkingMask();
      if (pose) drawPose(pose);
      if (objects) drawObjects(objects, objSize);
      if (faces) drawFaces(faces, faceSize);
    };

    faceRegisterBtn.addEventListener('click', async () => {
      const name = prompt('등록할 이름을 입력하세요');
      if (!name) return;
      setStatus(`Registering: ${name} ...`);
      try {
        const res = await fetch(`/face/register?name=${encodeURIComponent(name)}`);
        const text = await res.text();
        setStatus(text);
      } catch (e) {
        setStatus(`Register error: ${e}`);
      }
    });

    faceDeleteBtn.addEventListener('click', async () => {
      const name = prompt('삭제할 이름을 입력하세요');
      if (!name) return;
      setStatus(`Deleting: ${name} ...`);
      try {
        const res = await fetch(`/face/delete?name=${encodeURIComponent(name)}`);
        const text = await res.text();
        setStatus(text);
      } catch (e) {
        setStatus(`Delete error: ${e}`);
      }
    });
  </script>
</body>
</html>
"""
class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        _track_fps()
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/announce":
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0
            raw = self.rfile.read(length) if length > 0 else b""
            try:
                payload = json.loads(raw.decode("utf-8")) if raw else {}
            except Exception:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "Invalid JSON"}).encode("utf-8"))
                return
            text = str(payload.get("text", "")).strip()
            if not text:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "text is required"}).encode("utf-8"))
                return
            role = str(payload.get("role", "gate") or "gate").strip().lower()
            if role not in {"gate", "pinky"}:
                role = "gate"
            voice = str(payload.get("voice", "male") or "male").strip().lower()
            if voice not in {"male", "female", "auto"}:
                voice = "male"

            with _announce_lock:
                _announce_state["id"] += 1
                _announce_state["text"] = text
                _announce_state["role"] = role
                _announce_state["voice"] = voice
                _announce_state["ts"] = time.time()
                cur_id = _announce_state["id"]

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "id": cur_id, "role": role, "voice": voice}, ensure_ascii=False).encode("utf-8"))
            return

        if self.path == "/assistant":
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0
            raw = self.rfile.read(length) if length > 0 else b""
            try:
                payload = json.loads(raw.decode("utf-8")) if raw else {}
            except Exception:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "Invalid JSON"}).encode("utf-8"))
                return
            text = str(payload.get("text", "")).strip()
            recognized_name = str(payload.get("recognized_name", "")).strip()
            if not text:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "text is required"}).encode("utf-8"))
                return
            ok, data = assistant_decide(text, recognized_name=recognized_name)
            self.send_response(200 if ok else 503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            if ok:
                self.wfile.write(json.dumps({"ok": True, **data}, ensure_ascii=False).encode("utf-8"))
            else:
                self.wfile.write(json.dumps({"ok": False, "error": str(data)}, ensure_ascii=False).encode("utf-8"))
            return

        if self.path == "/chat":
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except Exception:
                length = 0
            raw = self.rfile.read(length) if length > 0 else b""
            try:
                payload = json.loads(raw.decode("utf-8")) if raw else {}
            except Exception:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "Invalid JSON"}).encode("utf-8"))
                return
            text = str(payload.get("text", "")).strip()
            if not text:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": "text is required"}).encode("utf-8"))
                return
            ok, msg = chat_reply(text)
            self.send_response(200 if ok else 503)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            if ok:
                self.wfile.write(json.dumps({"ok": True, "reply": msg}, ensure_ascii=False).encode("utf-8"))
            else:
                self.wfile.write(json.dumps({"ok": False, "error": msg}, ensure_ascii=False).encode("utf-8"))
            return
        self.send_error(404)
        self.end_headers()

    def do_GET(self):
        if self.path == "/":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
            return

        if self.path.startswith("/announce/latest"):
            with _announce_lock:
                data = {
                    "ok": True,
                    "id": int(_announce_state["id"]),
                    "text": str(_announce_state["text"]),
                    "role": str(_announce_state.get("role", "gate")),
                    "voice": str(_announce_state.get("voice", "male")),
                    "ts": float(_announce_state["ts"]),
                }
            content = json.dumps(data, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
            return

        if self.path.startswith("/emotion"):
            qs = parse_qs(urlparse(self.path).query)
            emo = (qs.get("name", [""])[0] or "").strip()
            if emo not in EMOTIONS:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"Invalid emotion")
                return

            ok, msg = set_emotion(emo)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/face/register"):
            qs = parse_qs(urlparse(self.path).query)
            name = (qs.get("name", [""])[0] or "").strip()
            persist_raw = (qs.get("persist", ["1"])[0] or "1").strip().lower()
            persist = persist_raw not in ("0", "false", "no", "off")
            ok, msg = register_face(name, persist=persist)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/face/list"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            names = [item.get("name") for item in _face_db if item.get("name")]
            self.wfile.write(json.dumps({"names": names}).encode("utf-8"))
            return

        if self.path.startswith("/face/delete"):
            qs = parse_qs(urlparse(self.path).query)
            name = (qs.get("name", [""])[0] or "").strip()
            ok, msg = delete_face(name)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/face/clear_temp"):
            ok, msg = clear_temp_faces()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/nav/home/set"):
            ok, msg = nav_set_home()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/nav/return/start"):
            ok, msg = nav_start_return()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/nav/return/stop"):
            ok, msg = nav_stop_return()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/nav/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(nav_status()).encode("utf-8"))
            return

        if self.path.startswith("/autonav/start"):
            qs = parse_qs(urlparse(self.path).query)
            mode = (qs.get("mode", ["person_follow"])[0] or "person_follow").strip()
            slot_raw = (qs.get("slot", [""])[0] or "").strip()
            start_cell_raw = (qs.get("start_cell", [""])[0] or "").strip()
            start_heading_raw = (qs.get("start_heading", [""])[0] or "").strip()
            reset_start_raw = (qs.get("reset_start", ["0"])[0] or "0").strip().lower()
            chain_phase2_raw = (qs.get("chain_phase2", ["0"])[0] or "0").strip().lower()
            chain_phase2_slot_raw = (qs.get("phase2_slot", [""])[0] or "").strip()
            chain_phase2_reset_raw = (qs.get("phase2_reset_start", ["1"])[0] or "1").strip().lower()
            slot = None
            start_cell = None
            start_heading = None
            reset_start = reset_start_raw in ("1", "true", "yes", "on")
            chain_phase2 = chain_phase2_raw in ("1", "true", "yes", "on")
            chain_phase2_slot = None
            chain_phase2_reset_start = chain_phase2_reset_raw in ("1", "true", "yes", "on")
            if slot_raw:
                try:
                    slot = int(slot_raw)
                except Exception:
                    slot = None
            if chain_phase2_slot_raw:
                try:
                    chain_phase2_slot = int(chain_phase2_slot_raw)
                except Exception:
                    chain_phase2_slot = None
            if start_cell_raw:
                try:
                    p = [v.strip() for v in start_cell_raw.split(",")]
                    if len(p) == 2:
                        start_cell = (int(p[0]), int(p[1]))
                except Exception:
                    start_cell = None
            if start_heading_raw:
                try:
                    start_heading = int(start_heading_raw)
                except Exception:
                    start_heading = None
            ok, msg = autonav_start(
                mode=mode,
                slot=slot,
                start_cell=start_cell,
                start_heading=start_heading,
                reset_start=reset_start,
                chain_phase2=chain_phase2,
                chain_phase2_slot=chain_phase2_slot,
                chain_phase2_reset_start=chain_phase2_reset_start,
            )
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/autonav/stop"):
            ok, msg = autonav_stop(reason="api")
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/autonav/phase2/reset_start"):
            ok, msg = autonav_phase2_reset_start_pose()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/autonav/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(autonav_status()).encode("utf-8"))
            return

        if self.path.startswith("/digit/start"):
            qs = parse_qs(urlparse(self.path).query)
            raw = (qs.get("id", [""])[0] or "").strip()
            try:
                marker_id = int(raw)
            except Exception:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"digit id is required (0~49)")
                return
            ok, msg = digit_start(marker_id)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/digit/stop"):
            ok, msg = digit_stop(reason="api")
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/digit/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(digit_status()).encode("utf-8"))
            return

        if self.path.startswith("/number/start"):
            qs = parse_qs(urlparse(self.path).query)
            raw = (qs.get("id", [""])[0] or "").strip()
            try:
                target_id = int(raw)
            except Exception:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"number id is required (1~4)")
                return
            ok, msg = number_start(target_id)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/number/stop"):
            ok, msg = number_stop(reason="api")
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/number/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(number_status()).encode("utf-8"))
            return

        if self.path.startswith("/cliff/start"):
            qs = parse_qs(urlparse(self.path).query)
            mode = (qs.get("mode", ["low"])[0] or "low").strip().lower()
            threshold_raw = (qs.get("threshold", [""])[0] or "").strip()
            threshold = None
            if threshold_raw:
                try:
                    threshold = float(threshold_raw)
                except Exception:
                    threshold = None
            ok, msg = cliff_guard_start(mode=mode, threshold=threshold)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/cliff/stop"):
            ok, msg = cliff_guard_stop()
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/cliff/status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(cliff_guard_status()).encode("utf-8"))
            return

        if self.path.startswith("/cmd_vel"):
            qs = parse_qs(urlparse(self.path).query)
            try:
                lin = float(qs.get("lin", ["0"])[0])
                ang = float(qs.get("ang", ["0"])[0])
            except ValueError:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b"Invalid cmd_vel")
                return
            manual_raw = (qs.get("manual", ["1"])[0] or "1").strip().lower()
            manual_override = manual_raw not in ("0", "false", "no", "off")
            ok, msg = publish_cmd_vel(lin, ang, manual_override=manual_override)
            self.send_response(200 if ok else 503)
            self.end_headers()
            self.wfile.write(msg.encode("utf-8"))
            return

        if self.path.startswith("/telemetry"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            with _vision_lock:
                pose = _vision_pose
                verr = _vision_err
                vts = _vision_last_ts
            with _obj_lock:
                objs = list(_obj_results) if _obj_results is not None else None
                osize = dict(_obj_size) if _obj_size is not None else None
                oerr = _obj_err
                ots = _obj_last_ts
            with _person_lock:
                ptracks = list(_person_tracks) if _person_tracks is not None else None
                psize = dict(_person_size) if _person_size is not None else None
                pts = _person_last_ts
            with _face_lock:
                faces = list(_face_results) if _face_results is not None else None
                fsize = dict(_face_size) if _face_size is not None else None
                ferr = _face_err
                fts = _face_last_ts
            nav = nav_status()
            autonav = autonav_status()
            digit = digit_status()
            number = number_status()
            cliff = cliff_guard_status()
            payload = {
                "battery": _battery_voltage,
                "us": _us_range,
                "scan_min": _scan_min_range,
                "scan_left": _scan_left_range,
                "scan_right": _scan_right_range,
                "scan_rear": _scan_rear_range,
                "odom_xy": _odom_xy,
                "odom_yaw": _odom_yaw,
                "odom_age": (time.time() - _odom_last_ts) if _odom_last_ts else None,
                "ir": _ir_ranges,
                "fps": _fps,
                "pose": pose,
                "vision_err": verr,
                "vision_age": (time.time() - vts) if vts else None,
                "objects": objs,
                "obj_size": osize,
                "obj_err": oerr,
                "obj_age": (time.time() - ots) if ots else None,
                "person_tracks": ptracks,
                "person_size": psize,
                "person_age": (time.time() - pts) if pts else None,
                "faces": faces,
                "face_size": fsize,
                "face_err": ferr,
                "face_age": (time.time() - fts) if fts else None,
                "greeting": _consume_greet_pending(),
                "nav": nav,
                "autonav": autonav,
                "digit": digit,
                "number": number,
                "cliff": cliff,
            }
            self.wfile.write(json.dumps(payload).encode("utf-8"))
            return

        if self.path == "/stream.mjpg":
            if output is None:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"Camera not available")
                return
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception as exc:
                logging.warning("Client disconnected: %s", exc)
            return

        self.send_error(404)
        self.end_headers()

    def log_message(self, format, *args):
        return


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


_ros_enabled = False
_ros_lock = threading.Lock()
_ros_node = None
_ros_client = None
_ros_pub = None
_ros_name_pub = None
_ros_name_last = None
_ros_name_last_ts = 0.0
_battery_voltage = None
_us_range = None
_us_last_ts = 0.0
_ir_ranges = None
_ir_last_ts = 0.0
_scan_min_range = None
_scan_left_range = None
_scan_right_range = None
_scan_rear_range = None
_scan_last_ts = 0.0
_odom_lock = threading.Lock()
_odom_xy = None
_odom_yaw = None
_odom_last_ts = 0.0
_led_client = None
_led_state = None
_led_last_ts = 0.0
_led_mode = "auto"
_led_min_interval = 1.5
_led_brightness = 1.0
_led_pending_state = None
_led_pending_count = 0
_ros_executor = None
_ros_thread = None
_fps = 0.0
_fps_frames = 0
_fps_last_ts = 0.0
_vision_lock = threading.Lock()
_vision_pose = None
_vision_err = None
_vision_last_ts = 0.0
_obj_lock = threading.Lock()
_obj_results = []
_obj_size = None
_obj_err = None
_obj_last_ts = 0.0
_person_lock = threading.Lock()
_person_tracks = []
_person_size = None
_person_last_ts = 0.0
_person_primary_id = 0
_person_track_next_id = 1
_person_min_score = 0.30
_person_track_iou = 0.28
_person_track_max_miss = 5
_person_track_max_tracks = 4
_person_track_smooth = 0.55
_face_lock = threading.Lock()
_face_app = None
_face_db = []
_face_temp_db = []
_face_db_path = None
_face_threshold = 0.35
_face_results = []
_face_err = None
_face_last_ts = 0.0
_face_size = None
_chat_lock = threading.Lock()
_chat_history = []
_chat_history_max = 6
_announce_lock = threading.Lock()
_announce_state = {"id": 0, "text": "", "role": "gate", "voice": "male", "ts": 0.0}
_frame_lock = threading.Lock()
_latest_frame = None
_latest_frame_ts = 0.0
_latest_frame_err = None
_nav_lock = threading.Lock()
_nav_home_set = False
_nav_returning = False
_nav_path = []
_nav_record_max = 800
_nav_last_cmd_ts = 0.0
_nav_last_lin = 0.0
_nav_last_ang = 0.0
_nav_segment_min = 0.05
_autonav_lock = threading.Lock()
_autonav_enabled = False
_autonav_mode = "person_follow"
_autonav_last_msg = "idle"
_autonav_last_target = ""
_autonav_last_seen_ts = 0.0
_autonav_thread = None
_autonav_thread_last_loop_ts = 0.0
_autonav_thread_last_err = ""
_autonav_cycle_hz = 6.0
_autonav_turn_gain = 2.2
_autonav_turn_max = 0.9
_autonav_search_ang = 0.55
_autonav_search_ang_slow = 0.28
_autonav_forward_speed = 0.24
_autonav_min_box_ratio = 0.06
_autonav_max_box_ratio = 0.18
_autonav_lost_timeout = 0.9
_autonav_companion_turn = 0.34
_autonav_companion_forward = 0.07
_autonav_companion_period = 3.2
_autonav_target_max_age = 0.45
_autonav_err_deadband = 0.05
_autonav_turn_alpha = 0.32
_autonav_cmd_hold_timeout = 0.35
_autonav_prev_target = None
_autonav_prev_target_ts = 0.0
_autonav_prev_lin = 0.0
_autonav_prev_ang = 0.0
_autonav_prev_cmd_ts = 0.0
_autonav_search_dir = 1.0
_autonav_search_flip_ts = 0.0
_autonav_search_flip_period = 1.0
_autonav_lane_state = "follow"
_autonav_lane_state_ts = 0.0
_autonav_lane_last_line_ts = 0.0
_autonav_lane_turn_count = 0
_autonav_lane_turn_target = 2
_autonav_lane_last_x = 0.0
_autonav_lane_last_err = 0.0
_autonav_lane_offset_mm = 50.0
_autonav_lane_bottom_width_mm = 115.6
_autonav_lane_follow_speed = 0.17
_autonav_lane_turn_speed = 0.62
_autonav_lane_turn_duration = 1.10
_autonav_lane_loss_trigger = 0.42
_autonav_lane_reacquire_timeout = 1.60
_autonav_lane_kp = 1.35
_autonav_lane_turn_max = 0.72
_autonav_lane_hsv_low = (6, 95, 70)
_autonav_lane_hsv_high = (28, 255, 255)
_autonav_lane_roi_top_ratio = 0.40
_autonav_lane_jump_px = 48.0
_autonav_lane_virtual_hold = 0.45
_autonav_lane_turn_gate_x = 205.0
_autonav_lane_reacquire_bias_ang = -0.16
_autonav_phase1_chain_to_phase2 = False
_autonav_phase1_chain_phase2_slot = 5
_autonav_phase1_chain_phase2_reset_start = True
_autonav_phase2_state = "plan_path"
_autonav_phase2_state_ts = 0.0
_autonav_phase2_done = False
_autonav_phase2_target_slot = 5
_autonav_phase2_current_cell = (5, 1)
_autonav_phase2_current_heading = 1  # 0:+x, 1:+y, 2:-x, 3:-y
_autonav_phase2_path_cells = []
_autonav_phase2_cmd_queue = []
_autonav_phase2_cmd_idx = 0
_autonav_phase2_step_idx = 1
_autonav_phase2_turn_dir = 0
_autonav_phase2_turn_remaining = 0
_autonav_phase2_move_start_xy = None
_autonav_phase2_move_start_yaw = None
_autonav_phase2_heading_ref_yaw = None  # yaw of heading +Y (index 1)
_autonav_phase2_pre_aligned = False
_autonav_phase2_pre_align_streak = 0
_autonav_phase2_right_line_ref_x = None
_autonav_phase2_parking_bbox = None
_autonav_phase2_parking_conf = 0.0
_autonav_phase2_guide_line = None  # (x1,y1,x2,y2) in full-frame image coords
_autonav_phase2_line_lock_deg = None
_autonav_phase2_goal_kind = "to_slot"  # "to_start" | "to_slot"
_autonav_phase2_stop_cell = None
_autonav_phase2_park_cell = None
_autonav_phase2_stop_align_streak = 0
_autonav_phase2_replan_count = 0
_autonav_phase2_obstacle_hits = 0
_autonav_phase2_temp_blocks = {}
_autonav_phase2_temp_block_ttl = 2.5
_autonav_phase2_replan_scan_stop = 0.28
_autonav_phase2_replan_hit_frames = 3
_autonav_phase2_replan_us_confirm = 0.22
_autonav_phase2_replan_min_elapsed = 0.35
_autonav_phase2_replan_parallel_tol_deg = 8.0
_autonav_phase2_replan_hard_scan_stop = 0.12
_autonav_phase2_replan_hard_us_stop = 0.10
_autonav_phase2_turn_side_stop = 0.16
_autonav_phase2_green_hsv_low = (36, 45, 40)
_autonav_phase2_green_hsv_high = (88, 255, 255)
_autonav_phase2_blue_hsv_low = (100, 120, 70)
_autonav_phase2_blue_hsv_high = (130, 255, 255)
_autonav_phase2_align_roi_top_ratio = 0.22
_autonav_phase2_boundary_roi_top_ratio = 0.40
_autonav_phase2_parking_roi_top_ratio = 0.28
_autonav_phase2_align_ok_streak = 0
_autonav_phase2_align_need_frames = 6
_autonav_phase2_align_tol_deg = 10.0
_autonav_phase2_align_orth_tol_deg = 14.0
_autonav_phase2_align_kp = 0.95
_autonav_phase2_align_max_ang = 0.42
_autonav_phase2_align_timeout = 3.5
_autonav_phase2_pre_align_rotate = False
_autonav_phase2_follow_jump_px = 28.0
_autonav_phase2_reverse_rear_stop = 0.10
_autonav_phase2_turn_speed = 0.62
_autonav_phase2_turn90_sec = 2.55
_autonav_phase2_linear_speed = 0.10
_autonav_phase2_reverse_speed = 0.10
_autonav_phase2_forward_heading_kp = 1.20
_autonav_phase2_forward_heading_max_ang = 0.32
_autonav_phase2_heading_lock_tol_deg = 5.0
_autonav_phase2_pre_align_timeout = 2.0
_autonav_phase2_pre_align_tol_deg = 6.0
_autonav_phase2_pre_align_need_frames = 5
_autonav_phase2_pre_align_kp = 0.90
_autonav_phase2_pre_align_max_ang = 0.35
_autonav_phase2_enable_pre_align = False
_autonav_phase2_forward_line_kp = 0.65
_autonav_phase2_forward_line_max_ang = 0.22
_autonav_phase2_right_line_kp = 0.62
_autonav_phase2_right_line_max_ang = 0.26
_autonav_phase2_right_line_min_ratio = 0.45
_autonav_phase2_right_line_use_fixed_ref = True
_autonav_phase2_right_line_target_m = 0.05   # desired line offset from robot center (trial value)
_autonav_phase2_camera_x_offset_m = 0.00     # phase2: keep direct 5cm like phase1 (no cam offset compensation)
_autonav_phase2_bottom_width_m = 0.1156      # calibrated floor width at image bottom (320px)
_autonav_phase2_slot_detect_enabled = False
_autonav_phase2_slot_idx = 1
_autonav_phase2_slot_last_cross_ts = 0.0
_autonav_phase2_slot_cross_cooldown_sec = 0.75
_autonav_phase2_slot_cross_band_y_ratio = 0.84
_autonav_phase2_slot_cross_band_half_px = 8
_autonav_phase2_slot_cross_min_blue_ratio = 0.16
_autonav_phase2_cell_m = 0.25
_autonav_phase2_odom_timeout = 0.8
_autonav_phase2_forward_time_gain = 1.38
_autonav_phase2_reverse_time_gain = 1.48
_autonav_phase2_move_time_bias_sec = 0.10
_autonav_phase2_keepout_y1_ratio = 0.70
_autonav_phase2_keepout_x1_ratio = 0.30
_autonav_phase2_keepout_x2_ratio = 0.78
_autonav_phase2_keepout_blue_ratio = 0.10
_autonav_phase2_right_keepout_y1_ratio = 0.66
_autonav_phase2_right_keepout_x1_ratio = 0.76
_autonav_phase2_right_keepout_blue_ratio = 0.18
_autonav_phase2_enable_keepout_guard = False
_autonav_phase2_use_phase1_logic = False
_autonav_phase2_use_blue_lane_follow = False
_autonav_phase2_force_last_slot = False
_autonav_phase2_grid_size = 6
_autonav_phase2_grid_min_x = 1
_autonav_phase2_grid_max_x = 6
_autonav_phase2_grid_min_y = 0
_autonav_phase2_grid_max_y = 6
_autonav_phase2_start_cell = (5, 0)
_autonav_phase2_exit_cell = (3, 1)
_autonav_phase2_exit_heading = 3  # -Y
_autonav_phase2_require_start_anchor = False
_autonav_phase2_blocks_raw = {
    (3.5, 2),
    (3.5, 4),
    (3, 6), (4, 6),
}
# Cell planner is integer-grid based.
# Half-cell pillars (x=3.5) are expanded to adjacent blocked cells.
_autonav_phase2_blocks = {
    (3, 2), (4, 2),
    (3, 4), (4, 4),
    (3, 6), (4, 6),
}
_autonav_phase2_park_map = {
    1: {"park": (6, 1), "stop": (5, 1)},
    2: {"park": (6, 2), "stop": (5, 2)},
    3: {"park": (6, 3), "stop": (5, 3)},
    4: {"park": (6, 4), "stop": (5, 4)},
    5: {"park": (6, 5), "stop": (5, 5)},
    6: {"park": (1, 5), "stop": (2, 5)},
    7: {"park": (1, 4), "stop": (2, 4)},
    8: {"park": (1, 3), "stop": (2, 3)},
    9: {"park": (1, 2), "stop": (2, 2)},
    10: {"park": (1, 1), "stop": (2, 1)},
}
_autonav_phase2_forward_cell_sec = max(
    0.2,
    (_autonav_phase2_cell_m / max(0.02, _autonav_phase2_linear_speed)) * _autonav_phase2_forward_time_gain
    + _autonav_phase2_move_time_bias_sec,
)
_autonav_phase2_reverse_cell_sec = max(
    0.2,
    (_autonav_phase2_cell_m / max(0.02, _autonav_phase2_reverse_speed)) * _autonav_phase2_reverse_time_gain
    + _autonav_phase2_move_time_bias_sec,
)
_autonav_phase2_safe_stop_us = 0.12
_autonav_phase2_row2_stop_us = 0.11
_digit_lock = threading.Lock()
_digit_enabled = False
_digit_target_id = -1
_digit_state = "idle"
_digit_last_msg = "idle"
_digit_last_seen_ts = 0.0
_digit_results = []
_digit_size = None
_digit_last_ts = 0.0
_digit_thread = None
_digit_cycle_hz = 5.0
_digit_turn_gain = 2.0
_digit_turn_max = 0.7
_digit_forward_speed = 0.18
_digit_stop_box_ratio = 0.10
_digit_arrive_box_ratio = 0.18
_digit_forward_min_scale = 0.60
_digit_marker_size_m = 0.16
_digit_arrive_dist_m = 0.45
_digit_arrive_ratio_fallback = 0.30
_digit_lost_timeout = 1.0
_digit_switch_backoff_sec = 0.8
_digit_backoff_speed = 0.10
_digit_search_ang = 0.35
_digit_reposition_until = 0.0
_digit_target_started_ts = 0.0
_digit_cam_fov_deg = 62.0
_digit_map_ttl = 45.0
_digit_heading_gain = 1.8
_digit_heading_deadband = 0.14
_digit_seek_forward_speed = 0.08
_digit_map = {}
_digit_yaw_est = 0.0
_digit_last_cmd_ang = 0.0
_digit_last_loop_ts = 0.0
_digit_search_dir = 1.0
_digit_search_flip_sec = 1.8
_digit_search_flip_ts = 0.0
_digit_error_count = 0
_digit_error_disable_threshold = 6
_number_lock = threading.Lock()
_number_enabled = False
_number_state = "idle"
_number_target_id = -1
_number_last_msg = "idle"
_number_last_seen_ts = 0.0
_number_results = []
_number_size = None
_number_last_ts = 0.0
_number_thread = None
_number_cycle_hz = 5.0
_number_turn_gain = 2.0
_number_turn_max = 0.7
_number_forward_speed = 0.20
_number_arrive_ratio = 0.20
_number_search_ang = 0.35
_number_lost_timeout = 0.8
_number_arrive_center_err = 0.12
_number_arrive_streak_need = 4
_number_map_ttl = 20.0
_number_cam_fov_deg = 62.0
_number_map_seek_forward_speed = 0.10
_number_map = {}
_number_yaw_est = 0.0
_number_last_cmd_ang = 0.0
_number_last_loop_ts = 0.0
_number_search_dir = 1.0
_number_search_flip_ts = 0.0
_number_search_flip_sec = 1.8
_number_arrive_streak = 0
_number_target_last_err = 0.0
_number_target_last_ratio = 0.0
_number_target_last_seen_ts = 0.0
_number_target_lock_sec = 0.9
_number_center_deadband = 0.08
_number_hold_forward_speed = 0.07
_number_ocr_enabled = True
_number_ocr_threshold = 0.40
_number_ocr_stride = 2
_safety_us_stop = 0.25
_safety_us_slow = 0.45
_safety_us_emergency_stop = 0.07
_safety_us_min_valid = 0.03
_safety_scan_stop = 0.20
_safety_scan_slow = 0.40
_safety_scan_emergency_stop = 0.07
_safety_scan_back_stop = 0.05
_safety_scan_back_slow = 0.40
_safety_scan_back_emergency_stop = 0.05
_safety_scan_min_valid = 0.10
_safety_scan_front_half_deg = 55.0
_safety_scan_center_deg = 0.0
_safety_scan_swap_lr = True
_safety_scan_swap_fb = False
_safety_sensor_timeout = 0.8
_safety_min_speed = 0.08
_safety_avoid_enabled = True
_safety_avoid_back_sec = 0.35
_safety_avoid_turn_sec = 0.55
_safety_avoid_back_speed = 0.08
_safety_avoid_turn_speed = 0.55
_safety_avoid_step_deg = 30.0
_safety_avoid_cooldown = 1.0
_safety_avoid_until = 0.0
_safety_avoid_back_until = 0.0
_safety_avoid_dir = 1.0
_safety_avoid_last_ts = 0.0
_safety_avoid_resume_until = 0.0
_safety_avoid_resume_sec = 0.60
_safety_avoid_resume_speed = 0.08
_safety_avoid_clear_scan = 0.24
_safety_avoid_clear_us = 0.20
_safety_avoid_lock = threading.Lock()
_cliff_guard_enabled = False
_cliff_guard_mode = "low"
_cliff_ir_threshold = 120.0
_cliff_sensor_timeout = 0.8
_reverse_speed_scale = 0.45
_reverse_turn_scale = 0.60
_reverse_align_time = 0.55
_reverse_align_speed = 0.08
_reverse_align_until = 0.0
_last_req_lin = 0.0
_cmd_state_lock = threading.Lock()
_greet_lock = threading.Lock()
_greet_state_path = None
_greet_day_key = ""
_greeted_today = {}
_greet_pending = ""
_lcd_default_text = "11"
_plate_number_text = "11"


def _ros_spin():
    while _ros_enabled and _ros_executor is not None:
        _ros_executor.spin_once(timeout_sec=0.1)


def init_ros():
    global _ros_enabled, _ros_node, _ros_client, _ros_pub, _ros_executor, _ros_thread, _led_client, _ros_name_pub
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from pinky_interfaces.srv import Emotion, SetLed
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Float32
        from std_msgs.msg import String
        from sensor_msgs.msg import Range, LaserScan
        from nav_msgs.msg import Odometry
        from std_msgs.msg import UInt16MultiArray
    except Exception:
        return False

    rclpy.init()
    _ros_node = rclpy.create_node("pinky_web_emotion")
    _ros_client = _ros_node.create_client(Emotion, "/set_emotion")
    if _led_mode != "off":
        _ros_node.create_timer(0.2, _led_timer_cb)
    _ros_pub = _ros_node.create_publisher(Twist, "/cmd_vel_joy", 10)
    _ros_name_pub = _ros_node.create_publisher(String, "/pinky/lcd/name", 10)
    _ros_node.create_subscription(Float32, "/battery/voltage", _battery_cb, 10)
    _ros_node.create_subscription(Range, "/us_sensor/range", _us_cb, 10)
    _ros_node.create_subscription(LaserScan, "/scan", _scan_cb, 10)
    _ros_node.create_subscription(Odometry, "/odom", _odom_cb, 10)
    _ros_node.create_subscription(UInt16MultiArray, "/ir_sensor/range", _ir_cb, 10)
    _led_client = _ros_node.create_client(SetLed, "/set_led")
    _ros_executor = SingleThreadedExecutor()
    _ros_executor.add_node(_ros_node)
    _ros_enabled = True
    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()
    return True


def shutdown_ros():
    global _ros_enabled
    if not _ros_enabled:
        return
    try:
        import rclpy
        _ros_enabled = False
        if _ros_executor and _ros_node:
            _ros_executor.remove_node(_ros_node)
        if _ros_node:
            _ros_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


def set_emotion(emo):
    if not _ros_enabled or _ros_client is None:
        return False, "ROS2 not available"

    with _ros_lock:
        if not _ros_client.wait_for_service(timeout_sec=0.5):
            return False, "Emotion service not available"
        try:
            from pinky_interfaces.srv import Emotion
        except Exception:
            return False, "Emotion service type unavailable"
        req = Emotion.Request()
        req.emotion = emo
        future = _ros_client.call_async(req)

    # wait a short time for response
    for _ in range(20):
        if future.done():
            try:
                resp = future.result()
                return True, resp.response or f"Set emotion: {emo}"
            except Exception as exc:
                return False, f"ROS2 call failed: {exc}"
        threading.Event().wait(0.05)

    return False, "Timeout waiting for ROS2 response"


def _nav_record_cmd(now_ts: float, lin: float, ang: float):
    global _nav_last_cmd_ts, _nav_last_lin, _nav_last_ang, _nav_path, _nav_returning
    with _nav_lock:
        # Manual command cancels auto return.
        if _nav_returning:
            _nav_returning = False
        # Always track last command timestamp to keep timing sane.
        if _nav_last_cmd_ts <= 0.0:
            _nav_last_cmd_ts = now_ts
            _nav_last_lin = float(lin)
            _nav_last_ang = float(ang)
            return
        dt = max(0.0, now_ts - _nav_last_cmd_ts)
        prev_lin = float(_nav_last_lin)
        prev_ang = float(_nav_last_ang)
        # Record only after home is set and only for moving segments.
        if _nav_home_set and dt >= _nav_segment_min and (abs(prev_lin) > 1e-3 or abs(prev_ang) > 1e-3):
            _nav_path.append(
                {
                    "lin": prev_lin,
                    "ang": prev_ang,
                    "dt": min(dt, 1.5),
                }
            )
            if len(_nav_path) > _nav_record_max:
                _nav_path = _nav_path[-_nav_record_max:]
        _nav_last_cmd_ts = now_ts
        _nav_last_lin = float(lin)
        _nav_last_ang = float(ang)


def nav_set_home():
    global _nav_home_set, _nav_path, _nav_last_cmd_ts, _nav_last_lin, _nav_last_ang, _nav_returning
    with _nav_lock:
        _nav_home_set = True
        _nav_returning = False
        _nav_path = []
        _nav_last_cmd_ts = time.time()
        _nav_last_lin = 0.0
        _nav_last_ang = 0.0
    return True, "출발점을 저장했습니다."


def nav_stop_return():
    global _nav_returning
    with _nav_lock:
        _nav_returning = False
    # Ensure robot stops when return mode is canceled.
    publish_cmd_vel(0.0, 0.0, record=False)
    return True, "복귀를 중지했습니다."


def _nav_return_worker(path_snapshot):
    global _nav_returning
    try:
        publish_cmd_vel(0.0, 0.0, record=False)
        time.sleep(0.05)
        for seg in reversed(path_snapshot):
            with _nav_lock:
                if not _nav_returning:
                    break
            lin = -float(seg.get("lin", 0.0))
            ang = -float(seg.get("ang", 0.0))
            dur = max(0.05, min(1.5, float(seg.get("dt", 0.0))))
            end_ts = time.time() + dur
            while time.time() < end_ts:
                with _nav_lock:
                    if not _nav_returning:
                        break
                ok, _ = publish_cmd_vel(lin, ang, record=False)
                if not ok:
                    with _nav_lock:
                        _nav_returning = False
                    break
                time.sleep(0.08)
            with _nav_lock:
                if not _nav_returning:
                    break
        publish_cmd_vel(0.0, 0.0, record=False)
    finally:
        with _nav_lock:
            _nav_returning = False


def nav_start_return():
    global _nav_returning, _autonav_enabled, _autonav_last_msg
    with _nav_lock:
        if not _nav_home_set:
            return False, "출발점을 먼저 저장하세요."
        if _nav_returning:
            return False, "이미 복귀 중입니다."
        if not _nav_path:
            return False, "복귀 경로가 없습니다."
        path_snapshot = list(_nav_path)
        _nav_returning = True
    with _autonav_lock:
        if _autonav_enabled:
            _autonav_enabled = False
            _autonav_last_msg = "stopped:return_home"
    t = threading.Thread(target=_nav_return_worker, args=(path_snapshot,), daemon=True)
    t.start()
    return True, "복귀를 시작합니다."


def nav_status():
    with _nav_lock:
        return {
            "home_set": bool(_nav_home_set),
            "returning": bool(_nav_returning),
            "path_len": len(_nav_path),
        }


def _clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, float(v)))


def _norm_angle(rad: float) -> float:
    return math.atan2(math.sin(float(rad)), math.cos(float(rad)))


def _box_ratio(box, w: float, h: float) -> float:
    if not box or w <= 1.0 or h <= 1.0:
        return 0.0
    bw = max(0.0, float(box[2]) - float(box[0]))
    bh = max(0.0, float(box[3]) - float(box[1]))
    return (bw * bh) / (float(w) * float(h))


def _target_center_err(box, w: float) -> float:
    if not box or w <= 1.0:
        return 1.0
    cx = (float(box[0]) + float(box[2])) * 0.5
    return abs((cx / float(w)) - 0.5)


def _bbox_iou(a, b) -> float:
    if not a or not b or len(a) < 4 or len(b) < 4:
        return 0.0
    ax1, ay1, ax2, ay2 = [float(v) for v in a[:4]]
    bx1, by1, bx2, by2 = [float(v) for v in b[:4]]
    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    aarea = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    barea = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    denom = (aarea + barea - inter)
    if denom <= 1e-6:
        return 0.0
    return inter / denom


def _update_person_tracks(person_dets, w: float, h: float):
    global _person_tracks, _person_size, _person_last_ts, _person_primary_id, _person_track_next_id
    now = time.time()
    dets = []
    for d in person_dets or []:
        box = d.get("box") if isinstance(d, dict) else None
        if not box or len(box) < 4:
            continue
        score = float(d.get("score", 0.0) or 0.0)
        if score < _person_min_score:
            continue
        dets.append({"box": [float(v) for v in box[:4]], "score": score})

    with _person_lock:
        tracks = [dict(t) for t in _person_tracks]
        used_tracks = set()
        used_dets = set()

        for ti, t in enumerate(tracks):
            best_j = -1
            best_iou = float(_person_track_iou)
            for dj, d in enumerate(dets):
                if dj in used_dets:
                    continue
                iou = _bbox_iou(t.get("box"), d.get("box"))
                if iou > best_iou:
                    best_iou = iou
                    best_j = dj
            if best_j < 0:
                continue
            used_tracks.add(ti)
            used_dets.add(best_j)
            d = dets[best_j]
            old = t.get("box") or d.get("box")
            a = max(0.0, min(1.0, float(_person_track_smooth)))
            sm = [((1.0 - a) * float(old[i])) + (a * float(d["box"][i])) for i in range(4)]
            t["box"] = sm
            t["score"] = d["score"]
            t["miss"] = 0
            t["hits"] = int(t.get("hits", 0) or 0) + 1
            t["ts"] = now
            tracks[ti] = t

        kept = []
        for ti, t in enumerate(tracks):
            if ti in used_tracks:
                kept.append(t)
                continue
            miss = int(t.get("miss", 0) or 0) + 1
            if miss <= int(_person_track_max_miss):
                t["miss"] = miss
                kept.append(t)

        for dj, d in enumerate(dets):
            if dj in used_dets:
                continue
            kept.append(
                {
                    "id": int(_person_track_next_id),
                    "box": d["box"],
                    "score": d["score"],
                    "miss": 0,
                    "hits": 1,
                    "ts": now,
                }
            )
            _person_track_next_id += 1

        kept.sort(
            key=lambda t: (
                int(t.get("miss", 99)),
                -float(t.get("score", 0.0)),
                -float(_box_ratio(t.get("box"), w, h)),
            )
        )
        kept = kept[: int(max(1, _person_track_max_tracks))]
        _person_tracks = kept
        _person_size = {"w": float(w), "h": float(h)}
        _person_last_ts = now

        if _person_primary_id:
            ids = [int(t.get("id", -1)) for t in kept]
            if int(_person_primary_id) not in ids:
                _person_primary_id = 0
        if not _person_primary_id and kept:
            _person_primary_id = int(kept[0].get("id", 0) or 0)


def _select_autonav_target():
    # Front-first targeting:
    # pick the most centered person in view (tie-break: larger box).
    now = time.time()
    candidates = []
    with _autonav_lock:
        prev = dict(_autonav_prev_target) if isinstance(_autonav_prev_target, dict) else None
        prev_age = (now - _autonav_prev_target_ts) if _autonav_prev_target_ts else 999.0

    with _person_lock:
        ptracks = list(_person_tracks) if _person_tracks else []
        psize = dict(_person_size) if _person_size else None
        pts = float(_person_last_ts or 0.0)
        primary_id = int(_person_primary_id or 0)
    if psize and ptracks and pts and ((now - pts) <= _autonav_target_max_age):
        pw = float(psize.get("w", 0) or 0)
        ph = float(psize.get("h", 0) or 0)
        primary = None
        for t in ptracks:
            if int(t.get("id", 0) or 0) == primary_id:
                primary = t
                break
        if primary is None:
            primary = ptracks[0]
        pbox = primary.get("box")
        if pbox and len(pbox) >= 4:
            return {
                "box": pbox[:4],
                "w": pw,
                "h": ph,
                "label": f"person_track#{int(primary.get('id', 0) or 0)}",
                "prio": -1,
            }

    with _face_lock:
        faces = list(_face_results) if _face_results else []
        fsize = dict(_face_size) if _face_size else None
        fts = float(_face_last_ts or 0.0)
    if fsize and faces and fts and ((now - fts) <= _autonav_target_max_age):
        fw = float(fsize.get("w", 0) or 0)
        fh = float(fsize.get("h", 0) or 0)
        for f in faces:
            box = f.get("box")
            if not box:
                continue
            label = f.get("name") or "face"
            candidates.append({"box": box, "w": fw, "h": fh, "label": str(label), "prio": 3})

    with _obj_lock:
        objs = list(_obj_results) if _obj_results else []
        osize = dict(_obj_size) if _obj_size else None
        ots = float(_obj_last_ts or 0.0)
    if osize and objs and ots and ((now - ots) <= _autonav_target_max_age):
        ow = float(osize.get("w", 0) or 0)
        oh = float(osize.get("h", 0) or 0)
        for o in objs:
            if not isinstance(o, dict):
                continue
            if str(o.get("label", "")).lower() != "person":
                continue
            box = o.get("box")
            if not box:
                continue
            candidates.append({"box": box, "w": ow, "h": oh, "label": "person", "prio": 1})

    with _vision_lock:
        pose = dict(_vision_pose) if isinstance(_vision_pose, dict) else None
        vts = float(_vision_last_ts or 0.0)
    if pose and vts and ((now - vts) <= _autonav_target_max_age):
        boxes = pose.get("boxes") or []
        if boxes:
            boxes = [b for b in boxes if isinstance(b, list) and len(b) >= 4]
            if boxes:
                w = float(pose.get("w", 0) or 0)
                h = float(pose.get("h", 0) or 0)
                for b in boxes:
                    candidates.append({"box": b[:4], "w": w, "h": h, "label": "pose_person", "prio": 4})

    if not candidates:
        return None

    if prev and prev_age <= 1.2:
        pbox = prev.get("box") if isinstance(prev, dict) else None
        candidates.sort(
            key=lambda c: (
                -_bbox_iou(c.get("box"), pbox),
                _target_center_err(c.get("box"), c.get("w", 0)),
                c.get("prio", 9),
                -_box_ratio(c.get("box"), c.get("w", 0), c.get("h", 0)),
            )
        )
    else:
        candidates.sort(
            key=lambda c: (
                _target_center_err(c.get("box"), c.get("w", 0)),
                c.get("prio", 9),
                -_box_ratio(c.get("box"), c.get("w", 0), c.get("h", 0)),
            )
        )
    return candidates[0]


def _grid_to_user(cell):
    if cell is None:
        return None
    try:
        return (int(cell[0]), int(cell[1]))
    except Exception:
        return None


def _grid_from_user(cell):
    if cell is None:
        return None
    try:
        return (int(cell[0]), int(cell[1]))
    except Exception:
        return None


def autonav_status():
    with _autonav_lock:
        worker_alive = bool(_autonav_thread is not None and _autonav_thread.is_alive())
        next_cell = None
        try:
            if _autonav_phase2_path_cells and 0 <= int(_autonav_phase2_step_idx) < len(_autonav_phase2_path_cells):
                next_cell = _grid_to_user(tuple(_autonav_phase2_path_cells[int(_autonav_phase2_step_idx)]))
        except Exception:
            next_cell = None
        path_user = []
        try:
            path_user = [_grid_to_user(c) for c in list(_autonav_phase2_path_cells)]
            path_user = [c for c in path_user if c is not None]
        except Exception:
            path_user = []
        return {
            "enabled": bool(_autonav_enabled),
            "mode": _autonav_mode,
            "last_msg": _autonav_last_msg,
            "worker_alive": worker_alive,
            "worker_last_loop_age": (time.time() - _autonav_thread_last_loop_ts) if _autonav_thread_last_loop_ts else None,
            "worker_last_err": _autonav_thread_last_err,
            "target": _autonav_last_target,
            "last_seen_age": (time.time() - _autonav_last_seen_ts) if _autonav_last_seen_ts else None,
            "lane_state": _autonav_lane_state,
            "lane_turn_count": int(_autonav_lane_turn_count),
            "lane_turn_target": int(_autonav_lane_turn_target),
            "phase1_chain_to_phase2": bool(_autonav_phase1_chain_to_phase2),
            "phase1_chain_phase2_slot": int(_autonav_phase1_chain_phase2_slot),
            "phase2_state": _autonav_phase2_state,
            "phase2_done": bool(_autonav_phase2_done),
            "phase2_target_slot": int(_autonav_phase2_target_slot),
            "phase2_path_cells": path_user,
            "phase2_current_cell": _grid_to_user(tuple(_autonav_phase2_current_cell)),
            "phase2_current_heading": int(_autonav_phase2_current_heading),
            "phase2_pre_aligned": bool(_autonav_phase2_pre_aligned),
            "phase2_pre_align_streak": int(_autonav_phase2_pre_align_streak),
            "phase2_right_line_ref_x": float(_autonav_phase2_right_line_ref_x) if _autonav_phase2_right_line_ref_x is not None else None,
            "phase2_parking_bbox": tuple(_autonav_phase2_parking_bbox) if _autonav_phase2_parking_bbox is not None else None,
            "phase2_parking_conf": float(_autonav_phase2_parking_conf),
            "phase2_guide_line": tuple(_autonav_phase2_guide_line) if _autonav_phase2_guide_line is not None else None,
            "phase2_right_line_fixed_ref": bool(_autonav_phase2_right_line_use_fixed_ref),
            "phase2_goal_kind": str(_autonav_phase2_goal_kind),
            "phase2_step_idx": int(_autonav_phase2_step_idx),
            "phase2_next_cell": next_cell,
            "phase2_stop_cell": _grid_to_user(tuple(_autonav_phase2_stop_cell)) if _autonav_phase2_stop_cell is not None else None,
            "phase2_park_cell": _grid_to_user(tuple(_autonav_phase2_park_cell)) if _autonav_phase2_park_cell is not None else None,
            "phase2_move_start_xy": tuple(_autonav_phase2_move_start_xy) if _autonav_phase2_move_start_xy else None,
            "phase2_heading_ref_yaw": float(_autonav_phase2_heading_ref_yaw) if _autonav_phase2_heading_ref_yaw is not None else None,
            "phase2_replan_count": int(_autonav_phase2_replan_count),
            "phase2_align_streak": int(_autonav_phase2_align_ok_streak),
            "phase2_slot_idx": int(_autonav_phase2_slot_idx),
            "phase2_slot_detect_enabled": bool(_autonav_phase2_slot_detect_enabled),
            "phase2_forward_cell_sec": float(_autonav_phase2_forward_cell_sec),
            "phase2_reverse_cell_sec": float(_autonav_phase2_reverse_cell_sec),
        }


def _autonav_reset_lane_phase_locked(now_ts: float):
    global _autonav_lane_state, _autonav_lane_state_ts, _autonav_lane_last_line_ts
    global _autonav_lane_turn_count, _autonav_lane_last_x, _autonav_lane_last_err
    _autonav_lane_state = "follow"
    _autonav_lane_state_ts = float(now_ts)
    _autonav_lane_last_line_ts = 0.0
    _autonav_lane_turn_count = 0
    _autonav_lane_last_x = 0.0
    _autonav_lane_last_err = 0.0


def _autonav_reset_phase2_locked(now_ts: float, keep_pose: bool = True):
    global _autonav_phase2_state, _autonav_phase2_state_ts, _autonav_phase2_done
    global _autonav_phase2_current_cell, _autonav_phase2_current_heading
    global _autonav_phase2_path_cells, _autonav_phase2_cmd_queue, _autonav_phase2_cmd_idx
    global _autonav_phase2_replan_count, _autonav_phase2_obstacle_hits, _autonav_phase2_temp_blocks
    global _autonav_phase2_align_ok_streak
    global _autonav_phase2_step_idx, _autonav_phase2_turn_dir, _autonav_phase2_turn_remaining
    global _autonav_phase2_move_start_xy, _autonav_phase2_move_start_yaw, _autonav_phase2_heading_ref_yaw
    global _autonav_phase2_pre_aligned, _autonav_phase2_pre_align_streak, _autonav_phase2_right_line_ref_x
    global _autonav_phase2_parking_bbox, _autonav_phase2_parking_conf, _autonav_phase2_guide_line, _autonav_phase2_line_lock_deg
    global _autonav_phase2_goal_kind
    global _autonav_phase2_stop_cell, _autonav_phase2_park_cell, _autonav_phase2_stop_align_streak
    global _autonav_phase2_slot_idx, _autonav_phase2_slot_last_cross_ts
    _autonav_phase2_state = "plan_path"
    _autonav_phase2_state_ts = float(now_ts)
    _autonav_phase2_done = False
    if (not keep_pose) or (not isinstance(_autonav_phase2_current_cell, tuple)) or (len(_autonav_phase2_current_cell) != 2):
        _autonav_phase2_current_cell = tuple(_autonav_phase2_start_cell)
    else:
        try:
            cx, cy = int(_autonav_phase2_current_cell[0]), int(_autonav_phase2_current_cell[1])
            if not (
                int(_autonav_phase2_grid_min_x) <= cx <= int(_autonav_phase2_grid_max_x)
                and int(_autonav_phase2_grid_min_y) <= cy <= int(_autonav_phase2_grid_max_y)
            ):
                _autonav_phase2_current_cell = tuple(_autonav_phase2_start_cell)
            else:
                _autonav_phase2_current_cell = (cx, cy)
        except Exception:
            _autonav_phase2_current_cell = tuple(_autonav_phase2_start_cell)

    try:
        h = int(_autonav_phase2_current_heading)
    except Exception:
        h = 1
    if h not in (0, 1, 2, 3):
        h = 1
    _autonav_phase2_current_heading = h
    _autonav_phase2_path_cells = []
    _autonav_phase2_cmd_queue = []
    _autonav_phase2_cmd_idx = 0
    _autonav_phase2_step_idx = 1
    _autonav_phase2_turn_dir = 0
    _autonav_phase2_turn_remaining = 0
    _autonav_phase2_move_start_xy = None
    _autonav_phase2_move_start_yaw = None
    _autonav_phase2_heading_ref_yaw = None
    _autonav_phase2_pre_aligned = False
    _autonav_phase2_pre_align_streak = 0
    _autonav_phase2_right_line_ref_x = None
    _autonav_phase2_parking_bbox = None
    _autonav_phase2_parking_conf = 0.0
    _autonav_phase2_guide_line = None
    _autonav_phase2_line_lock_deg = None
    _autonav_phase2_goal_kind = "to_slot"
    _autonav_phase2_stop_cell = None
    _autonav_phase2_park_cell = None
    _autonav_phase2_stop_align_streak = 0
    _autonav_phase2_replan_count = 0
    _autonav_phase2_obstacle_hits = 0
    _autonav_phase2_temp_blocks = {}
    _autonav_phase2_align_ok_streak = 0
    try:
        _autonav_phase2_slot_idx = int(max(1, min(10, int(_autonav_phase2_current_cell[1]))))
    except Exception:
        _autonav_phase2_slot_idx = 1
    _autonav_phase2_slot_last_cross_ts = 0.0


def _detect_green_features(frame, prev_right_x: float = None):
    try:
        import cv2
    except Exception:
        return None, 0.0, 0.0
    if frame is None:
        return None, 0.0, 0.0
    h, w = frame.shape[:2]
    if h < 24 or w < 24:
        return None, 0.0, 0.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo = np.array(_autonav_phase2_green_hsv_low, dtype=np.uint8)
    hi = np.array(_autonav_phase2_green_hsv_high, dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    kernel5 = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5, iterations=2)

    # Right boundary x estimate from lower ROI rows.
    y0 = int(h * 0.58)
    y1 = h - 1
    rows = [
        max(y0, y1 - 5),
        max(y0, y1 - 12),
        max(y0, y1 - 20),
        max(y0, y1 - 28),
    ]
    right_xs = []
    for ry in rows:
        row = mask[ry]
        xs = np.where(row > 0)[0]
        if xs.size >= 3:
            xs = xs[xs >= int(w * 0.45)]
            if xs.size >= 3:
                right_xs.append(float(np.median(xs)))
    right_x = None
    if right_xs:
        right_x = float(np.median(np.array(right_xs, dtype=np.float32)))
        if prev_right_x is not None and prev_right_x > 1.0 and abs(right_x - float(prev_right_x)) > 90.0:
            right_x = float(prev_right_x)

    # Front horizontal green confidence near horizon band.
    by0 = int(max(0, min(h - 1, h * 0.28)))
    by1 = int(max(by0 + 1, min(h, h * 0.48)))
    bx0 = int(w * 0.20)
    bx1 = int(max(bx0 + 1, w * 0.96))
    band = mask[by0:by1, bx0:bx1]
    front_ratio = 0.0
    if band.size > 0:
        front_ratio = float(np.count_nonzero(band)) / float(band.size)

    # Right side green occupancy helps detect boundary continuity.
    sy0 = int(max(0, min(h - 1, h * 0.45)))
    side = mask[sy0:h, int(w * 0.62):w]
    side_ratio = 0.0
    if side.size > 0:
        side_ratio = float(np.count_nonzero(side)) / float(side.size)

    return right_x, front_ratio, side_ratio


def _detect_phase2_align_angles(frame):
    try:
        import cv2
    except Exception:
        return None, None, 0.0
    if frame is None:
        return None, None, 0.0
    h, w = frame.shape[:2]
    if h < 32 or w < 32:
        return None, None, 0.0

    roi_top = int(max(0, min(h - 2, round(float(_autonav_phase2_align_roi_top_ratio) * h))))
    roi_bottom = max(roi_top + 1, h - 15)
    roi = frame[roi_top:roi_bottom, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    green_mask = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_green_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_green_hsv_high, dtype=np.uint8),
    )
    blue_mask = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_blue_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_blue_hsv_high, dtype=np.uint8),
    )
    blue_mask = cv2.bitwise_and(blue_mask, cv2.bitwise_not(green_mask))
    kernel = np.ones((5, 5), dtype=np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    def orientation_deg(mask):
        lines = cv2.HoughLinesP(
            mask,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=28,
            minLineLength=24,
            maxLineGap=12,
        )
        if lines is None or len(lines) == 0:
            return None, 0.0
        sum_c = 0.0
        sum_s = 0.0
        wsum = 0.0
        for ln in lines:
            x1, y1, x2, y2 = [float(v) for v in ln[0]]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < 8.0:
                continue
            ang = math.atan2(dy, dx)
            sum_c += math.cos(2.0 * ang) * length
            sum_s += math.sin(2.0 * ang) * length
            wsum += length
        if wsum <= 1e-6:
            return None, 0.0
        ori = 0.5 * math.atan2(sum_s, sum_c)
        deg = math.degrees(ori)
        if deg < 0.0:
            deg += 180.0
        cov = float(np.count_nonzero(mask)) / float(max(1, mask.size))
        return deg, cov

    green_deg, cg = orientation_deg(green_mask)
    blue_deg, cb = orientation_deg(blue_mask)
    conf = 0.5 * min(1.0, cg * 8.0) + 0.5 * min(1.0, cb * 8.0)
    return green_deg, blue_deg, conf


def _detect_phase2_boundary_x(frame, prev_x: float = None):
    global _autonav_phase2_guide_line
    try:
        import cv2
    except Exception:
        return None
    if frame is None:
        return None
    h, w = frame.shape[:2]
    if h < 24 or w < 24:
        return None
    _autonav_phase2_guide_line = None

    y0 = int(max(0, min(h - 1, round(float(_autonav_phase2_boundary_roi_top_ratio) * h))))
    roi = frame[y0:h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    gmask = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_green_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_green_hsv_high, dtype=np.uint8),
    )
    bmask = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_blue_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_blue_hsv_high, dtype=np.uint8),
    )
    bmask = cv2.bitwise_and(bmask, cv2.bitwise_not(gmask))
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    kernel5 = np.ones((5, 5), dtype=np.uint8)
    gmask = cv2.morphologyEx(gmask, cv2.MORPH_OPEN, kernel3, iterations=1)
    gmask = cv2.morphologyEx(gmask, cv2.MORPH_CLOSE, kernel5, iterations=2)
    bmask = cv2.morphologyEx(bmask, cv2.MORPH_OPEN, kernel3, iterations=1)
    bmask = cv2.morphologyEx(bmask, cv2.MORPH_CLOSE, kernel5, iterations=2)
    # If bottom-right ROI has too little blue, treat as "no valid boundary".
    br = bmask[int(roi.shape[0] * 0.55):, int(w * 0.52):]
    if br.size == 0:
        return None
    blue_ratio = float(np.count_nonzero(br)) / float(br.size)
    if blue_ratio < 0.012:
        return None

    def estimate_x(mask):
        # 1) Primary: choose the longest valid blue line as the driving axis.
        lines = cv2.HoughLinesP(
            mask,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=22,
            minLineLength=max(18, int(h * 0.14)),
            maxLineGap=10,
        )
        if lines is not None and len(lines) > 0:
            best_x = None
            best_len = -1.0
            best_score = -1e9
            for ln in lines:
                x1, y1, x2, y2 = [float(v) for v in ln[0]]
                dx = x2 - x1
                dy = y2 - y1
                length = math.hypot(dx, dy)
                if length < 10.0:
                    continue
                ang = abs(math.degrees(math.atan2(dy, dx)))
                # Keep slanted/vertical, reject almost-horizontal slot cross bars.
                if ang < 8.0 or ang > 88.0:
                    continue
                mx = (x1 + x2) * 0.5
                if mx < float(w) * 0.25:
                    continue
                # Need enough y-span so separator bars do not dominate.
                if abs(dy) < max(4.0, 0.10 * length):
                    continue
                # Extrapolate to near-bottom row for stable boundary x reference.
                y_ref = float(roi.shape[0] - 1)
                if abs(dy) > 1e-6:
                    t = (y_ref - y1) / dy
                    x_ref = x1 + (dx * t)
                else:
                    x_ref = mx
                if x_ref < float(w) * 0.22:
                    continue
                if x_ref > float(w) * 0.995:
                    continue
                right_bias = x_ref / max(1.0, float(w))
                continuity = abs(float(x_ref) - float(prev_x)) if (prev_x is not None and prev_x > 1.0) else 0.0
                score = (length * 1.0) + (right_bias * 35.0) - (continuity * 0.55) + (abs(dy) * 0.10)
                # Longest-line-first policy. Score is tie-breaker.
                if (length > best_len + 1e-3) or (abs(length - best_len) <= 1e-3 and score > best_score):
                    best_len = float(length)
                    best_score = score
                    best_x = float(x_ref)
                    # Save guide line in full-frame coordinates for overlay.
                    _autonav_phase2_guide_line = (
                        int(round(max(0.0, min(float(w - 1), x1)))),
                        int(round(max(0.0, min(float(h - 1), y1 + y0)))),
                        int(round(max(0.0, min(float(w - 1), x2)))),
                        int(round(max(0.0, min(float(h - 1), y2 + y0)))),
                    )
            if best_x is not None:
                best_x = max(0.0, min(float(w - 1), float(best_x)))
                if prev_x is not None and prev_x > 1.0 and abs(best_x - float(prev_x)) > float(_autonav_phase2_follow_jump_px):
                    return float(prev_x)
                return float(best_x)

        # 2) Fallback: contour-based estimate with vertical preference.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = None
        best_score = -1.0
        roi_h = roi.shape[0]
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < 120.0:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if ch < 14 or cw < 5:
                continue
            # Reject wide horizontal blobs (slot cross bars) and prefer vertical boundary-like shapes.
            aspect = float(ch) / max(1.0, float(cw))
            if aspect < 0.65:
                continue
            if (x + cw) < int(w * 0.35):
                continue
            cx = x + (0.5 * cw)
            right_bias = cx / max(1.0, float(w))
            continuity = 0.0
            if prev_x is not None and prev_x > 1.0:
                continuity = abs(cx - float(prev_x))
            score = (area * 0.52) + (ch * 10.0) + (right_bias * 72.0) + (aspect * 30.0) - (continuity * 1.0)
            if score > best_score:
                best_score = score
                best = cnt
        if best is None:
            return None

        # Use mid-to-lower rows but avoid the very bottom where horizontal slot bars dominate.
        rows = []
        for ratio in (0.58, 0.66, 0.74, 0.82):
            rows.append(max(0, min(roi_h - 1, int(roi_h * ratio))))
        row_xs = []
        for ry in rows:
            row = mask[ry]
            xs = np.where(row > 0)[0]
            if xs.size >= 4:
                xs = xs[xs >= int(w * 0.20)]
                if xs.size >= 4:
                    # Right boundary tracking: use right-most support, not median.
                    row_xs.append(float(np.percentile(xs, 85)))
        if row_xs:
            est = float(np.median(np.array(row_xs, dtype=np.float32)))
            if prev_x is not None and prev_x > 1.0 and abs(est - float(prev_x)) > float(_autonav_phase2_follow_jump_px):
                return float(prev_x)
            return est

        m = cv2.moments(best)
        if m.get("m00", 0.0) > 1e-6:
            est = float(m["m10"] / m["m00"])
            if prev_x is not None and prev_x > 1.0 and abs(est - float(prev_x)) > float(_autonav_phase2_follow_jump_px):
                return float(prev_x)
            return est
        return None

    # Parking lane guidance rule:
    # follow BLUE guideline only; never fallback to GREEN wall cue.
    bx = estimate_x(bmask)
    return bx


def _detect_phase2_slot_cross(frame):
    try:
        import cv2
    except Exception:
        return False
    if frame is None:
        return False
    h, w = frame.shape[:2]
    if h < 24 or w < 24:
        return False
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_blue_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_blue_hsv_high, dtype=np.uint8),
    )
    green = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_green_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_green_hsv_high, dtype=np.uint8),
    )
    blue = cv2.bitwise_and(blue, cv2.bitwise_not(green))
    kernel = np.ones((3, 3), dtype=np.uint8)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Slot index cue: watch a narrow horizontal band near the bottom-right lane.
    # A wide blue run in this band means we just crossed one slot separator.
    y0 = int(max(0, min(h - 1, round(float(_autonav_phase2_slot_cross_band_y_ratio) * h))))
    hh = int(max(2, float(_autonav_phase2_slot_cross_band_half_px)))
    y1 = max(0, y0 - hh)
    y2 = min(h, y0 + hh + 1)
    x1 = int(max(0, round(0.55 * w)))
    x2 = int(w)
    roi = blue[y1:y2, x1:x2]
    if roi.size == 0:
        return False
    blue_ratio = float(np.count_nonzero(roi)) / float(roi.size)
    return bool(blue_ratio >= float(_autonav_phase2_slot_cross_min_blue_ratio))


def _detect_phase2_parking_area(frame):
    try:
        import cv2
    except Exception:
        return None, 0.0
    if frame is None:
        return None, 0.0
    h, w = frame.shape[:2]
    if h < 24 or w < 24:
        return None, 0.0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_blue_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_blue_hsv_high, dtype=np.uint8),
    )
    green = cv2.inRange(
        hsv,
        np.array(_autonav_phase2_green_hsv_low, dtype=np.uint8),
        np.array(_autonav_phase2_green_hsv_high, dtype=np.uint8),
    )
    blue = cv2.bitwise_and(blue, cv2.bitwise_not(green))

    # Parking area is on floor; ignore top zone.
    roi_top = int(max(0, min(h - 1, round(float(_autonav_phase2_parking_roi_top_ratio) * h))))
    blue[:roi_top, :] = 0
    kernel = np.ones((5, 5), dtype=np.uint8)
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel, iterations=2)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0

    best = None
    best_score = 0.0
    frame_area = float(max(1, w * h))
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < 90.0:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if cw < 12 or ch < 6:
            continue
        # Parking shape cue: horizontal + vertical components mixed.
        aspect = float(cw) / max(1.0, float(ch))
        if aspect < 0.35 or aspect > 8.0:
            continue
        right_bias = (x + (0.5 * cw)) / max(1.0, float(w))
        score = (area / frame_area) * 2.0 + right_bias * 0.3 + min(1.0, (ch / max(1.0, h * 0.22))) * 0.2
        if score > best_score:
            best_score = score
            best = (int(x), int(y), int(cw), int(ch))

    if best is None:
        return None, 0.0
    return best, float(best_score)


def _autonav_cmd_parking_phase2():
    global _autonav_phase2_state, _autonav_phase2_state_ts, _autonav_phase2_done
    global _autonav_phase2_current_cell, _autonav_phase2_current_heading
    global _autonav_phase2_path_cells, _autonav_phase2_step_idx
    global _autonav_phase2_turn_dir, _autonav_phase2_turn_remaining, _autonav_phase2_move_start_xy, _autonav_phase2_move_start_yaw
    global _autonav_phase2_heading_ref_yaw
    global _autonav_phase2_pre_aligned, _autonav_phase2_pre_align_streak
    global _autonav_phase2_right_line_ref_x
    global _autonav_phase2_parking_bbox, _autonav_phase2_parking_conf, _autonav_phase2_guide_line, _autonav_phase2_line_lock_deg
    global _autonav_phase2_goal_kind
    global _autonav_phase2_stop_cell, _autonav_phase2_park_cell, _autonav_phase2_stop_align_streak
    global _autonav_phase2_slot_idx, _autonav_phase2_slot_last_cross_ts
    global _autonav_phase2_replan_count, _autonav_phase2_obstacle_hits, _autonav_phase2_temp_blocks

    now = time.time()
    with _autonav_lock:
        state = str(_autonav_phase2_state or "plan_path")
        state_ts = float(_autonav_phase2_state_ts or now)
        target_slot = int(_autonav_phase2_target_slot or 8)
        cur_cell = tuple(_autonav_phase2_current_cell)
        cur_heading = int(_autonav_phase2_current_heading)
        step_idx = int(_autonav_phase2_step_idx)
        goal_kind = str(_autonav_phase2_goal_kind or "to_slot")
        us = _us_range if (now - _us_last_ts) <= _safety_sensor_timeout else None
        scan_front = _scan_min_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
        scan_left = _scan_left_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
        scan_right = _scan_right_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
        scan_rear = _scan_rear_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
        slot_idx = int(_autonav_phase2_slot_idx)
        target_slot_locked = int(_autonav_phase2_target_slot or 5)
    with _odom_lock:
        odom_xy = tuple(_odom_xy) if _odom_xy is not None else None
        odom_yaw_now = float(_odom_yaw) if _odom_yaw is not None else None
        odom_age = (now - _odom_last_ts) if _odom_last_ts else None
    elapsed = max(0.0, now - state_ts)

    if bool(_autonav_phase2_use_blue_lane_follow):
        if state == "reverse_dock":
            if (scan_rear is not None) and (scan_rear <= float(_autonav_phase2_reverse_rear_stop)):
                with _autonav_lock:
                    _autonav_phase2_done = True
                    _autonav_phase2_state = "done"
                return 0.0, 0.0, f"phase2_reverse_done rear={scan_rear:.2f}", True

            with _autonav_lock:
                move_start = tuple(_autonav_phase2_move_start_xy) if _autonav_phase2_move_start_xy is not None else None
                if move_start is None and odom_xy is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
                    _autonav_phase2_move_start_xy = tuple(odom_xy)
                    move_start = tuple(odom_xy)
            moved = odom_move_dist(move_start, odom_xy)
            odom_ready = (moved is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout))
            reached = bool(moved >= float(_autonav_phase2_cell_m * 0.92)) if odom_ready else bool(elapsed >= float(_autonav_phase2_reverse_cell_sec))
            if reached:
                with _autonav_lock:
                    _autonav_phase2_done = True
                    _autonav_phase2_state = "done"
                if odom_ready:
                    return 0.0, 0.0, f"phase2_reverse_done dist={moved:.3f}", True
                return 0.0, 0.0, f"phase2_reverse_done time={elapsed:.2f}s", True
            return -abs(float(_autonav_phase2_reverse_speed)), 0.0, "phase2_reverse_dock", False

        target_slot = 5 if bool(_autonav_phase2_force_last_slot) else int(max(1, min(10, target_slot_locked)))
        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            with _autonav_lock:
                _autonav_phase2_parking_bbox = None
                _autonav_phase2_parking_conf = 0.0
                _autonav_phase2_guide_line = None
                _autonav_phase2_line_lock_deg = None
            return 0.0, 0.0, "phase2_wait_frame", False
        bbox, conf = _detect_phase2_parking_area(frame)
        with _autonav_lock:
            _autonav_phase2_parking_bbox = tuple(bbox) if bbox is not None else None
            _autonav_phase2_parking_conf = float(conf)
        # Parking-area bbox is now diagnostic only. Driving must continue using blue Y-axis boundary.

        # After parking area is found, follow blue boundary with fixed offset.
        fw = float(frame.shape[1])
        line_x = _detect_phase2_boundary_x(frame, prev_x=_autonav_phase2_right_line_ref_x)
        if line_x is None:
            if (scan_front is not None) and (scan_front <= 0.22):
                return 0.0, 0.22, f"phase2_no_blue_near_wall scan={scan_front:.2f}", False
            return 0.0, -0.18, f"phase2_search_blue_turn conf={conf:.3f}", False

        if bool(_autonav_phase2_slot_detect_enabled):
            crossed = _detect_phase2_slot_cross(frame)
            if crossed:
                with _autonav_lock:
                    if (now - float(_autonav_phase2_slot_last_cross_ts)) >= float(_autonav_phase2_slot_cross_cooldown_sec):
                        _autonav_phase2_slot_last_cross_ts = now
                        _autonav_phase2_slot_idx = int(max(1, min(10, int(_autonav_phase2_slot_idx) + 1)))
                        slot_idx = int(_autonav_phase2_slot_idx)

        if int(slot_idx) >= int(target_slot):
            with _autonav_lock:
                _autonav_phase2_done = False
                _autonav_phase2_state = "reverse_dock"
                _autonav_phase2_state_ts = now
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
            return 0.0, 0.0, f"phase2_arrived_front slot={int(slot_idx)} target={int(target_slot)} -> reverse", False

        with _autonav_lock:
            desired_cam_m = float(_autonav_phase2_right_line_target_m) - float(_autonav_phase2_camera_x_offset_m)
            px_per_m = fw / max(1e-6, float(_autonav_phase2_bottom_width_m))
            cx = 0.5 * fw
            ref_x = cx + (desired_cam_m * px_per_m)
            ref_x = max(0.0, min(fw - 1.0, float(ref_x)))
            _autonav_phase2_right_line_ref_x = float(ref_x)
            lock_deg = float(_autonav_phase2_line_lock_deg) if _autonav_phase2_line_lock_deg is not None else None

        err = (float(ref_x) - float(line_x)) / max(1.0, fw)
        if abs(err) < 0.03:
            err = 0.0
        ang_x = float(_autonav_phase2_right_line_kp) * float(err)
        ang_a = 0.0
        guide = _autonav_phase2_guide_line
        if isinstance(guide, tuple) and len(guide) >= 4:
            x1, y1, x2, y2 = [float(v) for v in guide[:4]]
            deg = math.degrees(math.atan2((y2 - y1), (x2 - x1)))
            while deg > 90.0:
                deg -= 180.0
            while deg < -90.0:
                deg += 180.0
            if lock_deg is None:
                with _autonav_lock:
                    if _autonav_phase2_line_lock_deg is None:
                        _autonav_phase2_line_lock_deg = float(deg)
                        lock_deg = float(deg)
                    else:
                        lock_deg = float(_autonav_phase2_line_lock_deg)
            if lock_deg is not None:
                d = deg - lock_deg
                while d > 90.0:
                    d -= 180.0
                while d < -90.0:
                    d += 180.0
                ang_a = _clamp((d / 45.0) * 0.18, -0.12, 0.12)
        ang = _clamp(
            float(ang_x + ang_a),
            -float(_autonav_phase2_right_line_max_ang),
            float(_autonav_phase2_right_line_max_ang),
        )
        # Never force zero steering from lateral offset alone.
        # Keep a minimal heading correction whenever line-angle error exists.
        if abs(ang_a) > 0.015 and abs(ang) < 0.05:
            ang = 0.05 if ang_a > 0.0 else -0.05
        lin_scale = _clamp(1.0 - (0.8 * abs(err)), 0.40, 1.0)
        lin = float(_autonav_phase2_linear_speed) * float(lin_scale)
        return float(lin), float(ang), f"phase2_to_front slot={int(slot_idx)} e={err:+.3f} a={ang_a:+.2f} conf={conf:.3f}", False

    def in_bounds(cell):
        x, y = cell
        return (
            int(_autonav_phase2_grid_min_x) <= int(x) <= int(_autonav_phase2_grid_max_x)
            and int(_autonav_phase2_grid_min_y) <= int(y) <= int(_autonav_phase2_grid_max_y)
        )

    def heading_for_step(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if dx == 1:
            return 0
        if dy == 1:
            return 1
        if dx == -1:
            return 2
        return 3

    def normalize_blocks():
        active = set()
        with _autonav_lock:
            for c, texp in list(_autonav_phase2_temp_blocks.items()):
                if now <= float(texp):
                    active.add(tuple(c))
                else:
                    _autonav_phase2_temp_blocks.pop(c, None)
        return active

    def free_cell(cell, goal=None):
        if not in_bounds(cell):
            return False
        if cell in _autonav_phase2_blocks:
            return False
        if goal is not None and tuple(cell) == tuple(goal):
            return True
        if cell in normalize_blocks():
            return False
        return True

    def neighbors(cell, goal=None):
        x, y = cell
        out = []
        for c in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if free_cell(c, goal=goal):
                out.append(c)
        return out

    def plan_path(start, goal):
        if not free_cell(start, goal=goal) or not free_cell(goal, goal=goal):
            return []
        frontier = []
        heapq.heappush(frontier, (0.0, start))
        came = {start: None}
        g_cost = {start: 0.0}
        while frontier:
            _, cur = heapq.heappop(frontier)
            if cur == goal:
                break
            for nxt in neighbors(cur, goal=goal):
                new_cost = g_cost[cur] + 1.0
                if (nxt not in g_cost) or (new_cost < g_cost[nxt]):
                    g_cost[nxt] = new_cost
                    h = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                    heapq.heappush(frontier, (new_cost + h, nxt))
                    came[nxt] = cur
        if goal not in came:
            return []
        out = []
        cur = goal
        while cur is not None:
            out.append(cur)
            cur = came[cur]
        return list(reversed(out))

    def init_turn(current_heading, target_heading):
        diff = (target_heading - current_heading) % 4
        if diff == 0:
            return 0, 0
        if diff == 1:
            return 1, 1
        if diff == 3:
            return -1, 1
        return 1, 2

    def wrap_pi(a):
        return math.atan2(math.sin(float(a)), math.cos(float(a)))

    def heading_to_yaw(heading_idx, ref_yaw_for_plus_y):
        # heading index: 0:+X, 1:+Y, 2:-X, 3:-Y
        # ref_yaw_for_plus_y is yaw for +Y axis.
        return wrap_pi(float(ref_yaw_for_plus_y) + ((int(heading_idx) - 1) * (math.pi * 0.5)))

    def odom_move_dist(start_xy, current_xy):
        if start_xy is None or current_xy is None:
            return None
        try:
            return math.hypot(float(current_xy[0]) - float(start_xy[0]), float(current_xy[1]) - float(start_xy[1]))
        except Exception:
            return None

    def line_axis_error_deg():
        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        g_deg, b_deg, conf = _detect_phase2_align_angles(frame)
        deg = b_deg if b_deg is not None else g_deg
        if deg is None:
            return None, 0.0
        d0 = abs(deg - 0.0)
        d90 = abs(deg - 90.0)
        target = 0.0 if d0 <= d90 else 90.0
        e = deg - target
        while e > 90.0:
            e -= 180.0
        while e < -90.0:
            e += 180.0
        return float(e), float(conf)

    if state == "plan_path":
        # Even when already at start cell, force +Y heading alignment before slot run.
        if tuple(cur_cell) == tuple(_autonav_phase2_start_cell) and int(cur_heading) != 1:
            with _autonav_lock:
                _autonav_phase2_goal_kind = "to_start"
                _autonav_phase2_state = "start_align"
                _autonav_phase2_state_ts = now
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
            return 0.0, 0.0, f"phase2_force_start_align heading={int(cur_heading)}", False

        with _autonav_lock:
            pre_aligned = bool(_autonav_phase2_pre_aligned)
        if bool(_autonav_phase2_enable_pre_align) and (not pre_aligned):
            with _autonav_lock:
                _autonav_phase2_state = "pre_align"
                _autonav_phase2_state_ts = now
                _autonav_phase2_pre_align_streak = 0
            return 0.0, 0.0, "phase2_pre_align_start", False
        if not bool(_autonav_phase2_enable_pre_align):
            with _autonav_lock:
                _autonav_phase2_pre_aligned = True

        slot_cfg = _autonav_phase2_park_map.get(target_slot)
        if not slot_cfg:
            with _autonav_lock:
                _autonav_phase2_state = "done"
                _autonav_phase2_done = True
            return 0.0, 0.0, f"phase2_invalid_slot:{target_slot}", True

        start = tuple(cur_cell)
        stop_cell = tuple(slot_cfg["stop"])
        park_cell = tuple(slot_cfg["park"])
        plan_goal_kind = str(goal_kind or "to_slot")
        goal = tuple(stop_cell)
        if bool(_autonav_phase2_require_start_anchor) and tuple(start) != tuple(_autonav_phase2_start_cell):
            plan_goal_kind = "to_start"
            goal = tuple(_autonav_phase2_start_cell)
        else:
            plan_goal_kind = "to_slot"
            goal = tuple(stop_cell)

        path = plan_path(start, goal)
        if len(path) < 2:
            with _autonav_lock:
                _autonav_phase2_state = "done"
                _autonav_phase2_done = True
            return 0.0, 0.0, f"phase2_plan_failed slot={target_slot} goal={plan_goal_kind}", True

        with _autonav_lock:
            _autonav_phase2_path_cells = list(path)
            _autonav_phase2_step_idx = 1
            _autonav_phase2_turn_dir = 0
            _autonav_phase2_turn_remaining = 0
            _autonav_phase2_move_start_xy = None
            _autonav_phase2_move_start_yaw = None
            _autonav_phase2_heading_ref_yaw = None
            _autonav_phase2_stop_cell = tuple(stop_cell)
            _autonav_phase2_park_cell = tuple(park_cell)
            _autonav_phase2_goal_kind = str(plan_goal_kind)
            _autonav_phase2_state = "exec_turn"
            _autonav_phase2_state_ts = now
        return 0.0, 0.0, f"phase2_plan_ok slot={target_slot} goal={plan_goal_kind} steps={len(path)}", False

    if state == "pre_align":
        e_deg, conf = line_axis_error_deg()
        with _autonav_lock:
            streak = int(_autonav_phase2_pre_align_streak)
        if e_deg is not None and conf >= 0.12:
            if abs(e_deg) <= float(_autonav_phase2_pre_align_tol_deg):
                streak += 1
            else:
                streak = 0
            with _autonav_lock:
                _autonav_phase2_pre_align_streak = streak
            if streak >= int(_autonav_phase2_pre_align_need_frames):
                with _autonav_lock:
                    _autonav_phase2_pre_aligned = True
                    _autonav_phase2_state = "plan_path"
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, "phase2_pre_align_done", False
            ang = _clamp(
                (float(e_deg) / 45.0) * float(_autonav_phase2_pre_align_kp),
                -float(_autonav_phase2_pre_align_max_ang),
                float(_autonav_phase2_pre_align_max_ang),
            )
            return 0.0, float(ang), f"phase2_pre_align e={e_deg:+.1f}", False

        if elapsed >= float(_autonav_phase2_pre_align_timeout):
            with _autonav_lock:
                _autonav_phase2_pre_aligned = True
                _autonav_phase2_state = "plan_path"
                _autonav_phase2_state_ts = now
            return 0.0, 0.0, "phase2_pre_align_timeout", False
        return 0.0, 0.0, "phase2_pre_align_wait_line", False

    if state == "exec_turn":
        with _autonav_lock:
            path = list(_autonav_phase2_path_cells)
            step_idx = int(_autonav_phase2_step_idx)
            turn_dir = int(_autonav_phase2_turn_dir)
            turn_rem = int(_autonav_phase2_turn_remaining)
            cur_cell = tuple(_autonav_phase2_current_cell)
            cur_heading = int(_autonav_phase2_current_heading)
            goal_kind = str(_autonav_phase2_goal_kind or "to_slot")
        if step_idx >= len(path):
            with _autonav_lock:
                _autonav_phase2_state = ("start_align" if goal_kind == "to_start" else "stop_align")
                _autonav_phase2_state_ts = now
            return 0.0, 0.0, ("phase2_to_start_align" if goal_kind == "to_start" else "phase2_to_stop_align"), False

        next_cell = tuple(path[step_idx])
        desired_heading = heading_for_step(cur_cell, next_cell)

        if turn_rem == 0:
            d, r = init_turn(cur_heading, desired_heading)
            with _autonav_lock:
                _autonav_phase2_turn_dir = int(d)
                _autonav_phase2_turn_remaining = int(r)
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
                _autonav_phase2_state_ts = now
            if r == 0:
                with _autonav_lock:
                    _autonav_phase2_state = "exec_forward_cell"
                    _autonav_phase2_move_start_xy = None
                    _autonav_phase2_move_start_yaw = None
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_turn_ok heading={desired_heading}", False
            return 0.0, (abs(_autonav_phase2_turn_speed) if d > 0 else -abs(_autonav_phase2_turn_speed)), f"phase2_turn_start dir={d} rem={r}", False

        if turn_dir > 0:
            if (scan_left is not None) and (scan_left <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_turn_hold:left_blocked={scan_left:.2f}", False
        else:
            if (scan_right is not None) and (scan_right <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_turn_hold:right_blocked={scan_right:.2f}", False

        if elapsed >= float(_autonav_phase2_turn90_sec):
            with _autonav_lock:
                if turn_dir > 0:
                    _autonav_phase2_current_heading = (cur_heading + 1) % 4
                else:
                    _autonav_phase2_current_heading = (cur_heading + 3) % 4
                _autonav_phase2_turn_remaining = max(0, turn_rem - 1)
                _autonav_phase2_state_ts = now
                if _autonav_phase2_turn_remaining == 0:
                    _autonav_phase2_state = "exec_forward_cell"
                    _autonav_phase2_move_start_xy = None
                    _autonav_phase2_move_start_yaw = None
            return 0.0, 0.0, "phase2_turn_step_done", False

        return 0.0, (abs(_autonav_phase2_turn_speed) if turn_dir > 0 else -abs(_autonav_phase2_turn_speed)), f"phase2_turning rem={turn_rem}", False

    if state == "exec_forward_cell":
        with _autonav_lock:
            path = list(_autonav_phase2_path_cells)
            step_idx = int(_autonav_phase2_step_idx)
            cur_cell = tuple(_autonav_phase2_current_cell)
            cur_heading = int(_autonav_phase2_current_heading)
            goal_kind = str(_autonav_phase2_goal_kind or "to_slot")
            stop_cell = tuple(_autonav_phase2_stop_cell) if _autonav_phase2_stop_cell is not None else None
            slot_idx = int(_autonav_phase2_slot_idx)
        if step_idx >= len(path):
            with _autonav_lock:
                _autonav_phase2_state = ("start_align" if goal_kind == "to_start" else "stop_align")
                _autonav_phase2_state_ts = now
            return 0.0, 0.0, ("phase2_to_start_align" if goal_kind == "to_start" else "phase2_to_stop_align"), False

        # Line-axis cue is used both for steering and for obstacle-gating.
        # If we are already well aligned to an orthogonal line, treat soft front warnings less aggressively.
        line_e_deg, line_conf = line_axis_error_deg()

        # Replan only on confirmed front obstacle.
        # This avoids false replans caused by single-sensor dropouts/noise at start.
        front_scan_blocked = (scan_front is not None) and (scan_front <= float(_autonav_phase2_replan_scan_stop))
        front_us_blocked = (us is not None) and (us <= float(_autonav_phase2_replan_us_confirm))
        front_scan_hard = (scan_front is not None) and (scan_front <= float(_autonav_phase2_replan_hard_scan_stop))
        front_us_hard = (us is not None) and (us <= float(_autonav_phase2_replan_hard_us_stop))
        blocked = (
            elapsed >= float(_autonav_phase2_replan_min_elapsed)
            and ((front_scan_blocked and front_us_blocked) or front_scan_hard or front_us_hard)
        )
        # Corridor lock for slots 1..5:
        # from (5,1)+Y to stop(5,y), do not allow soft replans to pull trajectory sideways/backwards.
        corridor_lock = False
        if (
            int(target_slot) in (1, 2, 3, 4, 5)
            and str(goal_kind) == "to_slot"
            and stop_cell is not None
            and int(cur_heading) == 1
            and int(cur_cell[0]) == 5
            and int(cur_cell[1]) < int(stop_cell[1])
        ):
            corridor_lock = True
        axis_aligned = (
            line_e_deg is not None
            and line_conf >= 0.12
            and abs(float(line_e_deg)) <= float(_autonav_phase2_replan_parallel_tol_deg)
        )
        if blocked and axis_aligned and (not front_scan_hard) and (not front_us_hard):
            blocked = False
        if corridor_lock and (not front_scan_hard) and (not front_us_hard):
            blocked = False
        if blocked:
            with _autonav_lock:
                _autonav_phase2_obstacle_hits += 1
                hits = int(_autonav_phase2_obstacle_hits)
            if hits >= int(_autonav_phase2_replan_hit_frames):
                ahead = {0: (1, 0), 1: (0, 1), 2: (-1, 0), 3: (0, -1)}.get(cur_heading, (0, 0))
                block_cell = (cur_cell[0] + ahead[0], cur_cell[1] + ahead[1])
                if in_bounds(block_cell):
                    with _autonav_lock:
                        _autonav_phase2_temp_blocks[tuple(block_cell)] = now + float(_autonav_phase2_temp_block_ttl)
                        _autonav_phase2_obstacle_hits = 0
                        _autonav_phase2_replan_count += 1
                        _autonav_phase2_move_start_xy = None
                        _autonav_phase2_move_start_yaw = None
                        _autonav_phase2_heading_ref_yaw = None
                        _autonav_phase2_state = "plan_path"
                        _autonav_phase2_state_ts = now
                    return 0.0, 0.0, f"phase2_replan obstacle@{block_cell}", False
        else:
            with _autonav_lock:
                _autonav_phase2_obstacle_hits = 0

        with _autonav_lock:
            move_start = tuple(_autonav_phase2_move_start_xy) if _autonav_phase2_move_start_xy is not None else None
            move_start_yaw = float(_autonav_phase2_move_start_yaw) if _autonav_phase2_move_start_yaw is not None else None
            if move_start is None and odom_xy is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
                _autonav_phase2_move_start_xy = tuple(odom_xy)
                move_start = tuple(odom_xy)
            if move_start_yaw is None and odom_yaw_now is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
                _autonav_phase2_move_start_yaw = float(odom_yaw_now)
                move_start_yaw = float(odom_yaw_now)
        moved = odom_move_dist(move_start, odom_xy)
        odom_ready = (moved is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout))
        reached_cell = bool(moved >= float(_autonav_phase2_cell_m)) if odom_ready else bool(elapsed >= float(_autonav_phase2_forward_cell_sec))

        ang_cmd = 0.0
        yaw_err_deg = None
        target_yaw = None
        if odom_yaw_now is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
            with _autonav_lock:
                ref_yaw = float(_autonav_phase2_heading_ref_yaw) if _autonav_phase2_heading_ref_yaw is not None else None
                if ref_yaw is None:
                    # Infer +Y reference from current yaw and current heading.
                    ref_yaw = wrap_pi(float(odom_yaw_now) - ((int(cur_heading) - 1) * (math.pi * 0.5)))
                    _autonav_phase2_heading_ref_yaw = float(ref_yaw)
            target_yaw = heading_to_yaw(cur_heading, ref_yaw)

        if target_yaw is not None and odom_yaw_now is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
            yaw_err = wrap_pi(target_yaw - odom_yaw_now)
            yaw_err_deg = yaw_err * (180.0 / math.pi)
            ang_cmd = _clamp(
                float(_autonav_phase2_forward_heading_kp) * float(yaw_err),
                -float(_autonav_phase2_forward_heading_max_ang),
                float(_autonav_phase2_forward_heading_max_ang),
            )

        if reached_cell:
            next_cell = tuple(path[step_idx])
            with _autonav_lock:
                _autonav_phase2_current_cell = tuple(next_cell)
                _autonav_phase2_step_idx = step_idx + 1
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
                _autonav_phase2_state_ts = now
                if _autonav_phase2_step_idx >= len(path):
                    _autonav_phase2_state = ("start_align" if goal_kind == "to_start" else "stop_align")
                    _autonav_phase2_stop_align_streak = 0
                else:
                    _autonav_phase2_state = "exec_turn"
            if odom_ready:
                if yaw_err_deg is not None:
                    return 0.0, 0.0, f"phase2_forward_done cell={next_cell} dist={moved:.3f} yaw_e={yaw_err_deg:+.1f}", False
                return 0.0, 0.0, f"phase2_forward_done cell={next_cell} dist={moved:.3f}", False
            return 0.0, 0.0, f"phase2_forward_done cell={next_cell} time={elapsed:.2f}s", False

        # Vision correction: always pull heading to nearest orthogonal axis (0/90 deg) from green/blue lines.
        if line_e_deg is not None and line_conf >= 0.12:
            line_ang = _clamp(
                (float(line_e_deg) / 45.0) * float(_autonav_phase2_forward_line_kp),
                -float(_autonav_phase2_forward_line_max_ang),
                float(_autonav_phase2_forward_line_max_ang),
            )
            ang_cmd = _clamp(
                float(ang_cmd) + float(line_ang),
                -float(_autonav_phase2_forward_heading_max_ang),
                float(_autonav_phase2_forward_heading_max_ang),
            )

        # Corridor rule for slots 1..5:
        # while heading +Y on x=5 lane, keep the right blue boundary (x=5.5 line)
        # at a target image x derived from desired real offset.
        right_line_e = None
        if int(target_slot) in (1, 2, 3, 4, 5) and int(cur_heading) == 1:
            with _frame_lock:
                frame = _latest_frame.copy() if _latest_frame is not None else None
            line_x = _detect_phase2_boundary_x(frame, prev_x=_autonav_phase2_right_line_ref_x)
            if frame is not None and line_x is not None:
                fw = max(1.0, float(frame.shape[1]))
                min_x = float(_autonav_phase2_right_line_min_ratio) * fw
                if float(line_x) >= min_x:
                    with _autonav_lock:
                        ref_x = None
                        if bool(_autonav_phase2_right_line_use_fixed_ref):
                            # Convert desired real lateral distance to fixed target pixel near bottom row.
                            # desired from camera = desired from robot center - camera lateral offset.
                            desired_cam_m = float(_autonav_phase2_right_line_target_m) - float(_autonav_phase2_camera_x_offset_m)
                            px_per_m = fw / max(1e-6, float(_autonav_phase2_bottom_width_m))
                            cx = 0.5 * fw
                            ref_x = cx + (desired_cam_m * px_per_m)
                            ref_x = max(0.0, min(fw - 1.0, float(ref_x)))
                            _autonav_phase2_right_line_ref_x = float(ref_x)
                        else:
                            ref_x = float(_autonav_phase2_right_line_ref_x) if _autonav_phase2_right_line_ref_x is not None else None
                            if ref_x is None:
                                _autonav_phase2_right_line_ref_x = float(line_x)
                                ref_x = float(line_x)
                    right_line_e = (float(ref_x) - float(line_x)) / fw
                    line_ang = _clamp(
                        float(_autonav_phase2_right_line_kp) * float(right_line_e),
                        -float(_autonav_phase2_right_line_max_ang),
                        float(_autonav_phase2_right_line_max_ang),
                    )
                    ang_cmd = _clamp(
                        float(ang_cmd) + float(line_ang),
                        -float(_autonav_phase2_forward_heading_max_ang),
                        float(_autonav_phase2_forward_heading_max_ang),
                    )
            if bool(_autonav_phase2_slot_detect_enabled) and bool(_autonav_phase2_use_blue_lane_follow) and frame is not None:
                crossed = _detect_phase2_slot_cross(frame)
                if crossed:
                    with _autonav_lock:
                        if (now - float(_autonav_phase2_slot_last_cross_ts)) >= float(_autonav_phase2_slot_cross_cooldown_sec):
                            _autonav_phase2_slot_last_cross_ts = now
                            _autonav_phase2_slot_idx = int(max(1, min(10, int(_autonav_phase2_slot_idx) + 1)))
                            slot_idx = int(_autonav_phase2_slot_idx)
                if int(target_slot) <= 5 and int(slot_idx) >= int(target_slot):
                    with _autonav_lock:
                        _autonav_phase2_state = "stop_align"
                        _autonav_phase2_state_ts = now
                        _autonav_phase2_stop_align_streak = 0
                        _autonav_phase2_move_start_xy = None
                        _autonav_phase2_move_start_yaw = None
                    return 0.0, 0.0, f"phase2_slot_guard stop@slot={slot_idx} target={int(target_slot)}", False

        if odom_ready:
            if yaw_err_deg is not None:
                if line_e_deg is not None:
                    if right_line_e is not None:
                        return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} yaw_e={yaw_err_deg:+.1f} line_e={line_e_deg:+.1f} rx_e={right_line_e:+.3f}{' lock' if corridor_lock else ''}", False
                    return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} yaw_e={yaw_err_deg:+.1f} line_e={line_e_deg:+.1f}{' lock' if corridor_lock else ''}", False
                if right_line_e is not None:
                    return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} yaw_e={yaw_err_deg:+.1f} rx_e={right_line_e:+.3f}{' lock' if corridor_lock else ''}", False
                return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} yaw_e={yaw_err_deg:+.1f}{' lock' if corridor_lock else ''}", False
            if line_e_deg is not None:
                if right_line_e is not None:
                    return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} line_e={line_e_deg:+.1f} rx_e={right_line_e:+.3f}{' lock' if corridor_lock else ''}", False
                return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} line_e={line_e_deg:+.1f}{' lock' if corridor_lock else ''}", False
            if right_line_e is not None:
                return float(_autonav_phase2_linear_speed), float(ang_cmd), f"phase2_forward cell={path[step_idx]} dist={moved:.3f} rx_e={right_line_e:+.3f}{' lock' if corridor_lock else ''}", False
            return float(_autonav_phase2_linear_speed), 0.0, f"phase2_forward cell={path[step_idx]} dist={moved:.3f}{' lock' if corridor_lock else ''}", False
        return float(_autonav_phase2_linear_speed), 0.0, f"phase2_forward cell={path[step_idx]} time={elapsed:.2f}s{' lock' if corridor_lock else ''}", False

    if state == "start_align":
        with _autonav_lock:
            cur_heading = int(_autonav_phase2_current_heading)
        desired_heading = 1  # +Y
        turn_dir, turn_rem = init_turn(cur_heading, desired_heading)
        if turn_rem > 0:
            if turn_dir > 0 and (scan_left is not None) and (scan_left <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_start_turn_hold:left={scan_left:.2f}", False
            if turn_dir < 0 and (scan_right is not None) and (scan_right <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_start_turn_hold:right={scan_right:.2f}", False
            if elapsed >= float(_autonav_phase2_turn90_sec):
                with _autonav_lock:
                    if turn_dir > 0:
                        _autonav_phase2_current_heading = (cur_heading + 1) % 4
                    else:
                        _autonav_phase2_current_heading = (cur_heading + 3) % 4
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, "phase2_start_turn_step", False
            return 0.0, (abs(_autonav_phase2_turn_speed) if turn_dir > 0 else -abs(_autonav_phase2_turn_speed)), "phase2_start_align_turning", False

        with _autonav_lock:
            _autonav_phase2_goal_kind = "to_slot"
            _autonav_phase2_state = "plan_path"
            _autonav_phase2_state_ts = now
            _autonav_phase2_move_start_xy = None
            _autonav_phase2_move_start_yaw = None
        return 0.0, 0.0, "phase2_start_anchor_ready", False

    if state == "stop_align":
        with _autonav_lock:
            stop_cell = tuple(_autonav_phase2_stop_cell) if _autonav_phase2_stop_cell is not None else None
            park_cell = tuple(_autonav_phase2_park_cell) if _autonav_phase2_park_cell is not None else None
            cur_heading = int(_autonav_phase2_current_heading)
            streak = int(_autonav_phase2_stop_align_streak)
        if stop_cell is None or park_cell is None:
            with _autonav_lock:
                _autonav_phase2_state = "done"
                _autonav_phase2_done = True
            return 0.0, 0.0, "phase2_invalid_stop_align", True

        park_dir = heading_for_step(stop_cell, park_cell)
        reverse_heading = (park_dir + 2) % 4
        turn_dir, turn_rem = init_turn(cur_heading, reverse_heading)
        if turn_rem > 0:
            # Reuse deterministic turn executor for stop heading setup.
            if turn_dir > 0 and (scan_left is not None) and (scan_left <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_stop_turn_hold:left={scan_left:.2f}", False
            if turn_dir < 0 and (scan_right is not None) and (scan_right <= float(_autonav_phase2_turn_side_stop)):
                with _autonav_lock:
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, f"phase2_stop_turn_hold:right={scan_right:.2f}", False

            if elapsed >= float(_autonav_phase2_turn90_sec):
                with _autonav_lock:
                    if turn_dir > 0:
                        _autonav_phase2_current_heading = (cur_heading + 1) % 4
                    else:
                        _autonav_phase2_current_heading = (cur_heading + 3) % 4
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, "phase2_stop_turn_step", False
            return 0.0, (abs(_autonav_phase2_turn_speed) if turn_dir > 0 else -abs(_autonav_phase2_turn_speed)), "phase2_stop_turning", False

        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        _green_deg, blue_deg, _conf = _detect_phase2_align_angles(frame)
        if blue_deg is None:
            # No blue line: use timeout fallback and keep still.
            if elapsed >= 0.7:
                with _autonav_lock:
                    _autonav_phase2_state = "reverse_dock"
                    _autonav_phase2_move_start_xy = None
                    _autonav_phase2_move_start_yaw = None
                    _autonav_phase2_state_ts = now
                return 0.0, 0.0, "phase2_stop_align_skip_no_blue", False
            return 0.0, 0.0, "phase2_stop_align_wait_blue", False

        e_blue = blue_deg - 0.0
        while e_blue > 90.0:
            e_blue -= 180.0
        while e_blue < -90.0:
            e_blue += 180.0

        if abs(e_blue) <= float(_autonav_phase2_align_tol_deg):
            streak += 1
        else:
            streak = 0
        with _autonav_lock:
            _autonav_phase2_stop_align_streak = streak

        if streak >= int(_autonav_phase2_align_need_frames) or elapsed >= float(_autonav_phase2_align_timeout):
            with _autonav_lock:
                _autonav_phase2_state = "reverse_dock"
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
                _autonav_phase2_state_ts = now
            return 0.0, 0.0, "phase2_stop_align_done", False

        ang = _clamp(
            (e_blue / 45.0) * float(_autonav_phase2_align_kp),
            -float(_autonav_phase2_align_max_ang),
            float(_autonav_phase2_align_max_ang),
        )
        return 0.0, float(ang), f"phase2_stop_align e={e_blue:+.1f}", False

    if state == "reverse_dock":
        if (scan_rear is not None) and (scan_rear <= float(_autonav_phase2_reverse_rear_stop)):
            with _autonav_lock:
                if _autonav_phase2_park_cell is not None:
                    _autonav_phase2_current_cell = tuple(_autonav_phase2_park_cell)
                _autonav_phase2_state = "done"
                _autonav_phase2_done = True
                _autonav_phase2_state_ts = now
            return 0.0, 0.0, f"phase2_done rear={scan_rear:.2f} target={float(_autonav_phase2_reverse_rear_stop):.2f}", True

        with _autonav_lock:
            move_start = tuple(_autonav_phase2_move_start_xy) if _autonav_phase2_move_start_xy is not None else None
            move_start_yaw = float(_autonav_phase2_move_start_yaw) if _autonav_phase2_move_start_yaw is not None else None
            if move_start is None and odom_xy is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
                _autonav_phase2_move_start_xy = tuple(odom_xy)
                move_start = tuple(odom_xy)
            if move_start_yaw is None and odom_yaw_now is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout):
                _autonav_phase2_move_start_yaw = float(odom_yaw_now)
        moved = odom_move_dist(move_start, odom_xy)
        odom_ready = (moved is not None and odom_age is not None and odom_age <= float(_autonav_phase2_odom_timeout))
        reached_cell = bool(moved >= float(_autonav_phase2_cell_m)) if odom_ready else bool(elapsed >= float(_autonav_phase2_reverse_cell_sec))

        if reached_cell:
            rear_now = float(scan_rear) if (scan_rear is not None) else None
            with _autonav_lock:
                if _autonav_phase2_park_cell is not None:
                    _autonav_phase2_current_cell = tuple(_autonav_phase2_park_cell)
                _autonav_phase2_move_start_xy = None
                _autonav_phase2_move_start_yaw = None
                _autonav_phase2_state = "done"
                _autonav_phase2_done = True
            if odom_ready:
                if rear_now is not None and rear_now > (float(_autonav_phase2_reverse_rear_stop) + 0.03):
                    return 0.0, 0.0, f"phase2_done reverse_dist={moved:.3f} rear={rear_now:.2f} target={float(_autonav_phase2_reverse_rear_stop):.2f} calc_warn", True
                return 0.0, 0.0, f"phase2_done reverse_dist={moved:.3f} rear={rear_now if rear_now is not None else -1:.2f}", True
            if rear_now is not None and rear_now > (float(_autonav_phase2_reverse_rear_stop) + 0.03):
                return 0.0, 0.0, f"phase2_done reverse_time={elapsed:.2f}s rear={rear_now:.2f} target={float(_autonav_phase2_reverse_rear_stop):.2f} calc_warn", True
            return 0.0, 0.0, f"phase2_done reverse_time={elapsed:.2f}s rear={rear_now if rear_now is not None else -1:.2f}", True

        if odom_ready:
            return -abs(float(_autonav_phase2_reverse_speed)), 0.0, f"phase2_reverse_dock dist={moved:.3f}", False
        return -abs(float(_autonav_phase2_reverse_speed)), 0.0, f"phase2_reverse_dock time={elapsed:.2f}s", False

    with _autonav_lock:
        _autonav_phase2_state = "done"
        _autonav_phase2_done = True
    return 0.0, 0.0, "phase2_done", True


def _detect_orange_line_x(frame, prev_x: float = None):
    try:
        import cv2
    except Exception:
        return None
    if frame is None:
        return None
    h, w = frame.shape[:2]
    if h < 16 or w < 16:
        return None

    y0 = int(max(0, min(h - 1, round(float(_autonav_lane_roi_top_ratio) * h))))
    roi = frame[y0:h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lo = np.array(_autonav_lane_hsv_low, dtype=np.uint8)
    hi = np.array(_autonav_lane_hsv_high, dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)
    kernel3 = np.ones((3, 3), dtype=np.uint8)
    kernel5 = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    best = None
    best_score = -1.0
    roi_h = roi.shape[0]
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < 120.0:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if ch < 14:
            continue
        if cw < 5:
            continue
        if (x + cw) < int(w * 0.25):
            continue
        # Prefer long/large right-side contours for wall stripe.
        cx = x + (0.5 * cw)
        right_bias = cx / max(1.0, float(w))
        continuity = 0.0
        if prev_x is not None and prev_x > 1.0:
            continuity = abs(cx - float(prev_x))
        score = (area * 0.75) + (ch * 8.0) + (right_bias * 70.0) - (continuity * 1.4)
        if score > best_score:
            best_score = score
            best = cnt
    if best is None:
        return None

    # Multi-row robust estimate to survive floor tile texture/noise.
    rows = [
        max(0, min(roi_h - 1, roi_h - 10)),
        max(0, min(roi_h - 1, roi_h - 16)),
        max(0, min(roi_h - 1, roi_h - 24)),
        max(0, min(roi_h - 1, roi_h - 32)),
    ]
    row_xs = []
    for ry in rows:
        row = mask[ry]
        xs = np.where(row > 0)[0]
        if xs.size >= 4:
            xs = xs[xs >= int(w * 0.20)]
            if xs.size >= 4:
                row_xs.append(float(np.median(xs)))
    if row_xs:
        est = float(np.median(np.array(row_xs, dtype=np.float32)))
        if prev_x is not None and prev_x > 1.0 and abs(est - float(prev_x)) > float(_autonav_lane_jump_px):
            return float(prev_x)
        return est

    m = cv2.moments(best)
    if m.get("m00", 0.0) > 1e-6:
        est = float(m["m10"] / m["m00"])
        if prev_x is not None and prev_x > 1.0 and abs(est - float(prev_x)) > float(_autonav_lane_jump_px):
            return float(prev_x)
        return est
    return None


def _autonav_cmd_lane_phase1():
    global _autonav_lane_state, _autonav_lane_state_ts, _autonav_lane_last_line_ts
    global _autonav_lane_turn_count, _autonav_lane_last_x, _autonav_lane_last_err
    now = time.time()
    with _autonav_lock:
        prev_x = float(_autonav_lane_last_x)
        prev_err = float(_autonav_lane_last_err)
        prev_line_ts = float(_autonav_lane_last_line_ts or 0.0)
    with _frame_lock:
        frame = _latest_frame.copy() if _latest_frame is not None else None
    line_x = _detect_orange_line_x(frame, prev_x=prev_x)
    has_line = line_x is not None
    virtual_line = False
    if (not has_line) and prev_x > 1.0 and prev_line_ts > 0.0 and (now - prev_line_ts) <= float(_autonav_lane_virtual_hold):
        line_x = prev_x
        has_line = True
        virtual_line = True
    w = float(frame.shape[1]) if frame is not None else 320.0

    with _autonav_lock:
        state = str(_autonav_lane_state)
        state_ts = float(_autonav_lane_state_ts or now)
        turn_count = int(_autonav_lane_turn_count)
        last_line_ts = float(_autonav_lane_last_line_ts or 0.0)
        had_line = last_line_ts > 0.0
        target_turns = int(_autonav_lane_turn_target)

        if has_line and (not virtual_line):
            _autonav_lane_last_line_ts = now
            _autonav_lane_last_x = float(line_x)
            _autonav_last_seen_ts = now
            _autonav_last_target = "orange_line"
            last_line_ts = now
        else:
            _autonav_last_target = ""

        if turn_count >= target_turns:
            _autonav_lane_state = "done"
            return 0.0, 0.0, f"phase1_done:{turn_count}", True

        if state == "turn_left":
            if (now - state_ts) < max(0.25, float(_autonav_lane_turn_duration)):
                return 0.0, abs(float(_autonav_lane_turn_speed)), f"phase1_turn_left:{turn_count + 1}", False
            turn_count += 1
            _autonav_lane_turn_count = turn_count
            if turn_count >= target_turns:
                _autonav_lane_state = "done"
                return 0.0, 0.0, f"phase1_done:{turn_count}", True
            _autonav_lane_state = "reacquire"
            _autonav_lane_state_ts = now
            return 0.08, 0.22, f"phase1_reacquire:{turn_count}", False

        if state == "reacquire":
            if has_line:
                _autonav_lane_state = "follow"
                _autonav_lane_state_ts = now
                state = "follow"
            elif (now - state_ts) > max(0.7, float(_autonav_lane_reacquire_timeout)):
                # Prevent premature left turns on tile-color noise.
                # Turn is allowed only when last confirmed line moved near image center/left.
                if had_line and float(_autonav_lane_last_x) <= float(_autonav_lane_turn_gate_x):
                    _autonav_lane_state = "turn_left"
                    _autonav_lane_state_ts = now
                    return 0.0, abs(float(_autonav_lane_turn_speed)), f"phase1_recover_turn:{turn_count + 1}", False
                _autonav_lane_state_ts = now
                return 0.06, float(_autonav_lane_reacquire_bias_ang), f"phase1_reacquire_extend:{turn_count}", False
            else:
                return 0.08, float(_autonav_lane_reacquire_bias_ang), f"phase1_search_line:{turn_count}", False

        if state == "follow":
            if not has_line:
                # Initial startup case: we have not locked onto any orange line yet.
                # Do not count turns before the first valid detection.
                if not had_line:
                    _autonav_lane_state = "reacquire"
                    _autonav_lane_state_ts = now
                    return 0.08, float(_autonav_lane_reacquire_bias_ang), f"phase1_start_search:{turn_count}", False
                lost = (now - last_line_ts) if last_line_ts > 0.0 else 999.0
                if lost >= max(0.15, float(_autonav_lane_loss_trigger)):
                    if float(_autonav_lane_last_x) <= float(_autonav_lane_turn_gate_x):
                        _autonav_lane_state = "turn_left"
                        _autonav_lane_state_ts = now
                        return 0.0, abs(float(_autonav_lane_turn_speed)), f"phase1_turn_left:{turn_count + 1}", False
                    _autonav_lane_state = "reacquire"
                    _autonav_lane_state_ts = now
                    return 0.08, float(_autonav_lane_reacquire_bias_ang), f"phase1_lost_but_hold:{turn_count}", False
                hold_ang = _clamp(prev_err * 0.60, -float(_autonav_lane_turn_max) * 0.5, float(_autonav_lane_turn_max) * 0.5)
                return 0.06, hold_ang, f"phase1_hold:{turn_count}", False

            mm_per_px = max(0.05, float(_autonav_lane_bottom_width_mm) / max(1.0, w))
            offset_px = float(_autonav_lane_offset_mm) / mm_per_px
            target_x = min(w - 6.0, max((w * 0.5) + 4.0, (w * 0.5) + offset_px))
            err = (target_x - float(line_x)) / max(1.0, w * 0.5)
            if abs(err) < 0.03:
                err = 0.0
            _autonav_lane_last_err = float(err)

            lin = float(_autonav_lane_follow_speed) * _clamp(1.0 - (0.70 * abs(err)), 0.35, 1.0)
            if abs(err) > 0.55:
                lin *= 0.45
            ang = _clamp(float(err) * float(_autonav_lane_kp), -float(_autonav_lane_turn_max), float(_autonav_lane_turn_max))
            suffix = "v" if virtual_line else "r"
            return lin, ang, f"phase1_follow:{suffix} t{turn_count} err={err:+.2f}", False

        _autonav_lane_state = "follow"
        _autonav_lane_state_ts = now
        return 0.0, 0.0, "phase1_reset", False


def autonav_stop(reason: str = "user"):
    global _autonav_enabled, _autonav_last_msg, _autonav_last_target
    global _autonav_prev_target, _autonav_prev_target_ts, _autonav_prev_lin, _autonav_prev_ang, _autonav_prev_cmd_ts
    global _autonav_search_dir, _autonav_search_flip_ts
    global _autonav_phase1_chain_to_phase2, _autonav_phase1_chain_phase2_slot, _autonav_phase1_chain_phase2_reset_start
    with _autonav_lock:
        was_enabled = bool(_autonav_enabled)
        _autonav_enabled = False
        _autonav_last_target = ""
        _autonav_last_msg = f"stopped:{reason}" if reason else "stopped"
        _autonav_prev_target = None
        _autonav_prev_target_ts = 0.0
        _autonav_prev_lin = 0.0
        _autonav_prev_ang = 0.0
        _autonav_prev_cmd_ts = 0.0
        _autonav_search_dir = 1.0
        _autonav_search_flip_ts = 0.0
        _autonav_phase1_chain_to_phase2 = False
        _autonav_phase1_chain_phase2_slot = 5
        _autonav_phase1_chain_phase2_reset_start = True
        _autonav_reset_lane_phase_locked(time.time())
        _autonav_reset_phase2_locked(time.time())
    publish_cmd_vel(0.0, 0.0, record=False)
    if was_enabled:
        return True, "자율주행을 중지했습니다."
    return True, "자율주행이 이미 중지 상태입니다."


def autonav_phase2_reset_start_pose():
    global _autonav_phase2_current_cell, _autonav_phase2_current_heading, _autonav_phase2_heading_ref_yaw
    global _autonav_phase2_pre_aligned, _autonav_phase2_pre_align_streak, _autonav_phase2_right_line_ref_x
    now = time.time()
    with _autonav_lock:
        if _autonav_enabled and str(_autonav_mode or "") == "parking_phase2":
            return False, "Phase2 실행 중에는 시작점 리셋 불가 (먼저 Stop)"
        _autonav_phase2_current_cell = tuple(_autonav_phase2_start_cell)
        _autonav_phase2_current_heading = 1
        _autonav_phase2_heading_ref_yaw = None
        _autonav_phase2_pre_aligned = False
        _autonav_phase2_pre_align_streak = 0
        _autonav_phase2_right_line_ref_x = None
        _autonav_reset_phase2_locked(now, keep_pose=True)
    return True, f"Phase2 pose reset: start={_grid_to_user(tuple(_autonav_phase2_current_cell))} heading={int(_autonav_phase2_current_heading)}"


def autonav_start(
    mode: str = "person_follow",
    slot: int = None,
    start_cell: tuple = None,
    start_heading: int = None,
    reset_start: bool = False,
    chain_phase2: bool = False,
    chain_phase2_slot: int = None,
    chain_phase2_reset_start: bool = True,
):
    global _autonav_enabled, _autonav_mode, _autonav_last_msg, _autonav_last_seen_ts, _autonav_thread
    global _autonav_prev_target, _autonav_prev_target_ts, _autonav_prev_lin, _autonav_prev_ang, _autonav_prev_cmd_ts
    global _autonav_search_dir, _autonav_search_flip_ts
    global _autonav_phase2_target_slot
    global _autonav_phase2_current_cell, _autonav_phase2_current_heading, _autonav_phase2_heading_ref_yaw
    global _autonav_phase1_chain_to_phase2, _autonav_phase1_chain_phase2_slot, _autonav_phase1_chain_phase2_reset_start
    mode = str(mode or "person_follow").strip().lower()
    if mode not in {"person_follow", "person_companion", "orange_lane_phase1", "parking_phase2"}:
        return False, f"지원하지 않는 모드입니다: {mode}"
    now = time.time()
    with _autonav_lock:
        if _autonav_enabled:
            return False, "이미 자율주행 중입니다."
        _autonav_enabled = True
        _autonav_mode = mode
        _autonav_last_msg = "started"
        _autonav_last_seen_ts = 0.0
        _autonav_prev_target = None
        _autonav_prev_target_ts = 0.0
        _autonav_prev_lin = 0.0
        _autonav_prev_ang = 0.0
        _autonav_prev_cmd_ts = 0.0
        _autonav_search_dir = 1.0
        _autonav_search_flip_ts = 0.0
        _autonav_phase1_chain_to_phase2 = False
        _autonav_phase1_chain_phase2_slot = 5
        _autonav_phase1_chain_phase2_reset_start = True
        if mode == "orange_lane_phase1":
            next_slot = 5 if chain_phase2_slot is None else int(chain_phase2_slot)
            if next_slot not in _autonav_phase2_park_map:
                next_slot = 5
            _autonav_phase1_chain_to_phase2 = bool(chain_phase2)
            _autonav_phase1_chain_phase2_slot = int(next_slot)
            _autonav_phase1_chain_phase2_reset_start = bool(chain_phase2_reset_start)
        if mode == "parking_phase2":
            target_slot = 5 if slot is None else int(slot)
            if bool(_autonav_phase2_force_last_slot):
                target_slot = 5
            if target_slot not in _autonav_phase2_park_map:
                target_slot = 5
            _autonav_phase2_target_slot = int(target_slot)
            if bool(reset_start):
                sc = tuple(_autonav_phase2_start_cell)
                sh = 1
                if isinstance(start_cell, tuple) and len(start_cell) == 2:
                    try:
                        internal_sc = _grid_from_user(start_cell)
                        sx = int(internal_sc[0])
                        sy = int(internal_sc[1])
                        if (
                            int(_autonav_phase2_grid_min_x) <= sx <= int(_autonav_phase2_grid_max_x)
                            and int(_autonav_phase2_grid_min_y) <= sy <= int(_autonav_phase2_grid_max_y)
                        ):
                            sc = (sx, sy)
                    except Exception:
                        pass
                try:
                    hh = int(start_heading) if start_heading is not None else 1
                    if hh in (0, 1, 2, 3):
                        sh = hh
                except Exception:
                    pass
                _autonav_phase2_current_cell = tuple(sc)
                _autonav_phase2_current_heading = int(sh)
                _autonav_phase2_heading_ref_yaw = None
        _autonav_reset_lane_phase_locked(now)
        _autonav_reset_phase2_locked(now)
    if _autonav_thread is None or not _autonav_thread.is_alive():
        _autonav_thread = threading.Thread(target=_autonav_worker, daemon=True)
        _autonav_thread.start()
    if mode == "orange_lane_phase1":
        with _autonav_lock:
            ch = bool(_autonav_phase1_chain_to_phase2)
            ch_slot = int(_autonav_phase1_chain_phase2_slot)
        if ch:
            return True, f"오렌지 라인 전반부 주행을 시작합니다. 완료 후 Phase2 자동 전환(slot={ch_slot})."
        return True, "오렌지 라인 전반부 주행을 시작합니다."
    if mode == "parking_phase2":
        with _autonav_lock:
            c = tuple(_autonav_phase2_current_cell)
            h = int(_autonav_phase2_current_heading)
        return True, f"주차장 2단계 주행을 시작합니다. target_slot={int(_autonav_phase2_target_slot)} start={_grid_to_user(c)} heading={h}"
    if mode == "person_companion":
        return True, "동행 모드를 시작합니다."
    return True, "추종 모드를 시작합니다."


def _autonav_cmd_from_target(target, mode: str = "person_follow"):
    box = target.get("box") or []
    w = float(target.get("w", 0) or 0)
    h = float(target.get("h", 0) or 0)
    if len(box) < 4 or w <= 1.0 or h <= 1.0:
        return 0.0, _autonav_search_ang, "invalid_target"

    x1, y1, x2, y2 = [float(v) for v in box[:4]]
    cx = (x1 + x2) * 0.5
    err = (cx / w) - 0.5  # -0.5(left) .. +0.5(right)
    if abs(err) < _autonav_err_deadband:
        err = 0.0
    ratio = _box_ratio([x1, y1, x2, y2], w, h)
    ang = _clamp(-err * _autonav_turn_gain, -_autonav_turn_max, _autonav_turn_max)

    if abs(err) > 0.35:
        lin = 0.0
    elif ratio < _autonav_min_box_ratio:
        gap = (_autonav_min_box_ratio - ratio) / max(_autonav_min_box_ratio, 1e-6)
        lin = _autonav_forward_speed * _clamp(0.55 + (0.45 * gap), 0.25, 1.0)
        if abs(err) > 0.20:
            lin *= 0.65
    elif ratio > _autonav_max_box_ratio:
        lin = -0.08
    else:
        if mode == "person_companion":
            # Gentle side-to-side motion around the person while keeping the face in view.
            phase = (time.time() % _autonav_companion_period) / max(0.5, _autonav_companion_period)
            sweep = math.sin(phase * math.pi * 2.0)
            lin = _autonav_companion_forward * max(0.0, 1.0 - (abs(err) / 0.20))
            ang = _clamp(
                (-err * _autonav_turn_gain) + (_autonav_companion_turn * sweep),
                -_autonav_turn_max,
                _autonav_turn_max,
            )
        else:
            lin = 0.0

    desc = f"{mode}:{target.get('label', 'target')} err={err:+.2f} ratio={ratio:.3f}"
    return lin, ang, desc


def _autonav_worker():
    global _autonav_enabled, _autonav_last_msg, _autonav_last_target, _autonav_last_seen_ts
    global _autonav_mode
    global _autonav_prev_target, _autonav_prev_target_ts, _autonav_prev_lin, _autonav_prev_ang, _autonav_prev_cmd_ts
    global _autonav_search_dir, _autonav_search_flip_ts
    global _autonav_phase2_target_slot, _autonav_phase1_chain_to_phase2
    global _autonav_thread_last_loop_ts, _autonav_thread_last_err
    interval = 1.0 / max(1.0, float(_autonav_cycle_hz))
    while True:
        try:
            loop_ts = time.time()
            with _autonav_lock:
                _autonav_thread_last_loop_ts = loop_ts
                enabled = bool(_autonav_enabled)
                mode = str(_autonav_mode or "person_follow")
            if not enabled:
                time.sleep(0.1)
                continue

            if not _ros_enabled or _ros_pub is None:
                with _autonav_lock:
                    _autonav_enabled = False
                    _autonav_last_msg = "ROS2 not available"
                time.sleep(0.2)
                continue

            if mode == "orange_lane_phase1":
                cmd_lin, cmd_ang, desc, done = _autonav_cmd_lane_phase1()
                if done:
                    with _autonav_lock:
                        chain_to_phase2 = bool(_autonav_phase1_chain_to_phase2)
                        chain_slot = int(_autonav_phase1_chain_phase2_slot)
                        chain_reset_start = bool(_autonav_phase1_chain_phase2_reset_start)
                    if chain_to_phase2:
                        now = time.time()
                        with _autonav_lock:
                            if chain_slot not in _autonav_phase2_park_map:
                                chain_slot = 5
                            _autonav_phase2_target_slot = int(chain_slot)
                            if chain_reset_start:
                                _autonav_phase2_current_cell = tuple(_autonav_phase2_start_cell)
                                _autonav_phase2_current_heading = 1
                                _autonav_phase2_heading_ref_yaw = None
                            _autonav_reset_phase2_locked(now, keep_pose=True)
                            _autonav_mode = "parking_phase2"
                            _autonav_last_msg = f"phase1_done -> phase2_start slot={int(_autonav_phase2_target_slot)}"
                            _autonav_prev_lin = 0.0
                            _autonav_prev_ang = 0.0
                            _autonav_prev_cmd_ts = now
                            _autonav_last_target = "phase1_complete"
                            _autonav_phase1_chain_to_phase2 = False
                        publish_cmd_vel(0.0, 0.0, record=False)
                        time.sleep(max(0.0, interval * 0.5))
                        continue
                    with _autonav_lock:
                        _autonav_enabled = False
                        _autonav_last_msg = desc
                        _autonav_prev_lin = 0.0
                        _autonav_prev_ang = 0.0
                        _autonav_prev_cmd_ts = time.time()
                        _autonav_last_target = "phase1_complete"
                    publish_cmd_vel(0.0, 0.0, record=False)
                    time.sleep(max(0.0, interval * 0.5))
                    continue

                ok, msg = publish_cmd_vel(cmd_lin, cmd_ang, record=False)
                now = time.time()
                with _autonav_lock:
                    _autonav_prev_lin = float(cmd_lin)
                    _autonav_prev_ang = float(cmd_ang)
                    _autonav_prev_cmd_ts = now
                    if ok and (not msg or msg == "OK"):
                        _autonav_last_msg = desc
                    else:
                        _autonav_last_msg = msg or desc
                elapsed = time.time() - loop_ts
                time.sleep(max(0.0, interval - elapsed))
                continue

            if mode == "parking_phase2":
                cmd_lin, cmd_ang, desc, done = _autonav_cmd_parking_phase2()
                if done:
                    with _autonav_lock:
                        _autonav_enabled = False
                        _autonav_last_msg = desc
                        _autonav_prev_lin = 0.0
                        _autonav_prev_ang = 0.0
                        _autonav_prev_cmd_ts = time.time()
                        _autonav_last_target = "phase2_complete"
                    publish_cmd_vel(0.0, 0.0, record=False)
                    time.sleep(max(0.0, interval * 0.5))
                    continue

                ok, msg = publish_cmd_vel(cmd_lin, cmd_ang, record=False)
                now = time.time()
                with _autonav_lock:
                    _autonav_prev_lin = float(cmd_lin)
                    _autonav_prev_ang = float(cmd_ang)
                    _autonav_prev_cmd_ts = now
                    if ok and (not msg or msg == "OK"):
                        _autonav_last_msg = desc
                    else:
                        _autonav_last_msg = msg or desc
                elapsed = time.time() - loop_ts
                time.sleep(max(0.0, interval - elapsed))
                continue

            target = _select_autonav_target()
            if target:
                lin, ang, desc = _autonav_cmd_from_target(target, mode=mode)
                with _autonav_lock:
                    prev_lin = float(_autonav_prev_lin)
                    prev_ang = float(_autonav_prev_ang)
                alpha = _clamp(float(_autonav_turn_alpha), 0.0, 1.0)
                cmd_lin = ((1.0 - alpha) * prev_lin) + (alpha * lin)
                cmd_ang = ((1.0 - alpha) * prev_ang) + (alpha * ang)
                ok, msg = publish_cmd_vel(cmd_lin, cmd_ang, record=False)
                now = time.time()
                with _autonav_lock:
                    _autonav_last_seen_ts = now
                    _autonav_last_target = str(target.get("label") or "target")
                    _autonav_prev_target = {
                        "box": list(target.get("box") or []),
                        "w": float(target.get("w", 0) or 0),
                        "h": float(target.get("h", 0) or 0),
                        "label": str(target.get("label") or "target"),
                    }
                    _autonav_prev_target_ts = now
                    _autonav_prev_lin = float(cmd_lin)
                    _autonav_prev_ang = float(cmd_ang)
                    _autonav_prev_cmd_ts = now
                    if ok and (not msg or msg == "OK"):
                        _autonav_last_msg = f"tracking:{desc}"
                    else:
                        _autonav_last_msg = msg or "tracking_failed"
            else:
                with _autonav_lock:
                    age = (time.time() - _autonav_last_seen_ts) if _autonav_last_seen_ts else 999.0
                now = time.time()
                with _autonav_lock:
                    cmd_age = (now - _autonav_prev_cmd_ts) if _autonav_prev_cmd_ts else 999.0
                    hold_lin = float(_autonav_prev_lin)
                    hold_ang = float(_autonav_prev_ang)
                if age > _autonav_lost_timeout:
                    with _autonav_lock:
                        if abs(hold_ang) > 0.06:
                            _autonav_search_dir = -1.0 if hold_ang > 0.0 else 1.0
                        if (now - _autonav_search_flip_ts) >= max(0.3, float(_autonav_search_flip_period)):
                            _autonav_search_dir *= -1.0
                            _autonav_search_flip_ts = now
                        search_ang = float(_autonav_search_ang_slow) * float(_autonav_search_dir)
                    ok, msg = publish_cmd_vel(0.0, search_ang, record=False)
                    status = "searching_target"
                elif cmd_age <= _autonav_cmd_hold_timeout:
                    ok, msg = publish_cmd_vel(hold_lin * 0.7, hold_ang * 0.7, record=False)
                    status = "holding_target"
                else:
                    ok, msg = publish_cmd_vel(0.0, 0.0, record=False)
                    status = "waiting_target"
                with _autonav_lock:
                    _autonav_last_target = ""
                    if age > _autonav_lost_timeout:
                        _autonav_prev_target = None
                        _autonav_prev_target_ts = 0.0
                    if ok and (not msg or msg == "OK"):
                        _autonav_last_msg = status
                    else:
                        _autonav_last_msg = msg or status

            elapsed = time.time() - loop_ts
            time.sleep(max(0.0, interval - elapsed))
        except Exception as exc:
            with _autonav_lock:
                _autonav_enabled = False
                _autonav_last_target = ""
                _autonav_thread_last_err = f"{type(exc).__name__}: {exc}"
                _autonav_last_msg = f"worker_err:{_autonav_thread_last_err}"
            publish_cmd_vel(0.0, 0.0, record=False)
            time.sleep(0.2)
            continue


def digit_status():
    with _digit_lock:
        now = time.time()
        known_ids = []
        ordered_pairs = []
        for mid, info in _digit_map.items():
            ts = float(info.get("seen_ts", 0.0)) if isinstance(info, dict) else 0.0
            if ts > 0.0 and (now - ts) <= max(1.0, float(_digit_map_ttl)):
                known_ids.append(int(mid))
                bearing = float(info.get("bearing", 0.0)) if isinstance(info, dict) else 0.0
                ordered_pairs.append((bearing, int(mid)))
        known_ids.sort()
        ordered_pairs.sort(key=lambda x: x[0])
        ordered_ids_lr = [mid for _, mid in ordered_pairs]
        return {
            "enabled": bool(_digit_enabled),
            "state": str(_digit_state),
            "target_id": int(_digit_target_id),
            "last_msg": str(_digit_last_msg),
            "last_seen_age": (time.time() - _digit_last_seen_ts) if _digit_last_seen_ts else None,
            "count": len(_digit_results),
            "size": dict(_digit_size) if isinstance(_digit_size, dict) else None,
            "known_ids": known_ids,
            "ordered_ids_lr": ordered_ids_lr,
            "error_count": int(_digit_error_count),
        }


def digit_stop(reason: str = "user"):
    global _digit_enabled, _digit_state, _digit_last_msg, _digit_reposition_until, _digit_last_cmd_ang
    with _digit_lock:
        was_enabled = bool(_digit_enabled)
        _digit_enabled = False
        _digit_state = "idle"
        _digit_reposition_until = 0.0
        _digit_last_cmd_ang = 0.0
        _digit_last_msg = f"stopped:{reason}" if reason else "stopped"
    publish_cmd_vel(0.0, 0.0, record=False)
    if was_enabled:
        return True, "숫자 이동을 중지했습니다."
    return True, "숫자 이동이 이미 중지 상태입니다."


def digit_start(marker_id: int):
    global _digit_enabled, _digit_target_id, _digit_state, _digit_last_msg, _digit_thread, _digit_reposition_until, _digit_target_started_ts
    global _digit_search_dir, _digit_search_flip_ts
    global _digit_error_count
    if marker_id < 0 or marker_id > 49:
        return False, "digit id must be 0~49"
    now = time.time()
    with _digit_lock:
        prev_enabled = bool(_digit_enabled)
        prev_target_id = int(_digit_target_id)
        _digit_enabled = True
        _digit_state = "scan"
        _digit_target_id = int(marker_id)
        _digit_error_count = 0
        if prev_enabled and prev_target_id != int(marker_id):
            _digit_reposition_until = now + max(0.0, float(_digit_switch_backoff_sec))
            _digit_last_msg = f"reposition_for_digit#{marker_id}"
        else:
            _digit_reposition_until = 0.0
            _digit_last_msg = "started"
        _digit_target_started_ts = now
        _digit_search_flip_ts = now
        if prev_target_id != int(marker_id):
            _digit_search_dir = 1.0
    if _digit_thread is None or not _digit_thread.is_alive():
        _digit_thread = threading.Thread(target=_digit_worker, daemon=True)
        _digit_thread.start()
    return True, f"숫자 {marker_id}로 이동을 시작합니다."


def _digit_worker():
    global _digit_enabled, _digit_state, _digit_last_msg, _digit_last_seen_ts, _digit_results, _digit_size, _digit_last_ts
    global _digit_map, _digit_yaw_est, _digit_last_cmd_ang, _digit_last_loop_ts
    global _digit_reposition_until, _digit_search_dir, _digit_search_flip_ts
    global _digit_error_count
    interval = 1.0 / max(1.0, float(_digit_cycle_hz))
    try:
        import cv2
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
    except Exception as exc:
        with _digit_lock:
            _digit_enabled = False
            _digit_last_msg = f"digit_init_failed:{exc}"
        return

    while True:
        loop_ts = time.time()
        with _digit_lock:
            enabled = bool(_digit_enabled)
            target_id = int(_digit_target_id)
            reposition_until = float(_digit_reposition_until)
            target_started_ts = float(_digit_target_started_ts)
            prev_loop_ts = float(_digit_last_loop_ts or 0.0)
            prev_cmd_ang = float(_digit_last_cmd_ang)
        if not enabled:
            time.sleep(0.1)
            continue

        now = time.time()
        dt = (now - prev_loop_ts) if prev_loop_ts > 0.0 else 0.0
        if dt > 0.0:
            with _digit_lock:
                _digit_yaw_est = _norm_angle(_digit_yaw_est + (prev_cmd_ang * dt))
                _digit_last_loop_ts = now
        else:
            with _digit_lock:
                _digit_last_loop_ts = now

        if reposition_until > now:
            ok, safety = publish_cmd_vel(-abs(float(_digit_backoff_speed)), 0.0, record=False)
            with _digit_lock:
                _digit_state = "scan"
                _digit_last_cmd_ang = 0.0
                _digit_last_msg = (safety if safety and safety != "OK" else f"repositioning:digit#{target_id}") if ok else (safety or "reposition_failed")
            elapsed = time.time() - loop_ts
            time.sleep(max(0.0, interval - elapsed))
            continue
        elif reposition_until > 0.0:
            with _digit_lock:
                _digit_reposition_until = 0.0

        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            time.sleep(0.1)
            continue

        try:
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            detections = []
            target = None
            now = time.time()
            fov_rad = math.radians(max(20.0, min(140.0, float(_digit_cam_fov_deg))))

            if ids is not None:
                for i in range(len(ids)):
                    mid = int(ids[i][0])
                    pts = corners[i][0]
                    x1 = float(min(p[0] for p in pts))
                    y1 = float(min(p[1] for p in pts))
                    x2 = float(max(p[0] for p in pts))
                    y2 = float(max(p[1] for p in pts))
                    bw = max(1.0, x2 - x1)
                    bh = max(1.0, y2 - y1)
                    ratio = max(0.0, ((x2 - x1) * (y2 - y1)) / max(1.0, float(w * h)))
                    # Approximate target distance from marker pixel size and camera HFOV.
                    # d ~= f * marker_size / pixel_size
                    f_px = (float(w) * 0.5) / max(1e-3, math.tan(fov_rad * 0.5))
                    px_size = 0.5 * (bw + bh)
                    dist_m = (f_px * max(0.01, float(_digit_marker_size_m))) / max(1.0, px_size)
                    det = {
                        "id": mid,
                        "box": [x1, y1, x2, y2],
                        "ratio": ratio,
                        "dist_m": float(dist_m),
                    }
                    detections.append(det)
                    cx = (x1 + x2) * 0.5
                    err = (cx / max(1.0, float(w))) - 0.5
                    # Camera x error -> robot yaw bearing.
                    # Use the same sign convention as tracking ang = -err * gain.
                    bearing = _norm_angle(_digit_yaw_est - (err * fov_rad))
                    with _digit_lock:
                        prev = _digit_map.get(mid, {})
                    prev_ratio = float(prev.get("ratio", 0.0)) if isinstance(prev, dict) else 0.0
                    if ratio >= prev_ratio * 0.9:
                        with _digit_lock:
                            _digit_map[mid] = {
                                "bearing": float(bearing),
                                "seen_ts": now,
                                "ratio": ratio,
                            }
                    if mid == target_id:
                        if target is None or ratio > float(target.get("ratio", 0.0)):
                            target = det

            # Cleanup stale map entries to keep memory bounded.
            ttl = max(1.0, float(_digit_map_ttl))
            with _digit_lock:
                stale = [mid for mid, info in _digit_map.items() if (now - float(info.get("seen_ts", 0.0))) > ttl]
                for mid in stale:
                    _digit_map.pop(mid, None)

            with _digit_lock:
                _digit_results = detections
                _digit_size = {"w": int(w), "h": int(h)}
                _digit_last_ts = time.time()

            if target is not None:
                cx = (target["box"][0] + target["box"][2]) * 0.5
                err = (cx / max(1.0, float(w))) - 0.5
                if abs(err) < 0.05:
                    err = 0.0
                ang = _clamp(-err * _digit_turn_gain, -_digit_turn_max, _digit_turn_max)
                ratio = float(target.get("ratio", 0.0))
                dist_m = float(target.get("dist_m", 0.0))
                arrived = False
                if dist_m > 0.0:
                    arrived = dist_m <= float(_digit_arrive_dist_m)
                else:
                    arrived = ratio >= float(_digit_arrive_ratio_fallback)
                if arrived:
                    lin = 0.0
                    msg = f"arrived:digit#{target_id} d={dist_m:.2f}m"
                elif abs(err) > 0.35:
                    lin = 0.0
                    msg = f"aligning:digit#{target_id} d={dist_m:.2f}m"
                else:
                    min_lin = max(0.03, float(_digit_forward_speed) * float(_digit_forward_min_scale))
                    speed_base = float(_digit_forward_speed)
                    if dist_m > float(_digit_arrive_dist_m):
                        speed_base = min(0.32, speed_base * 1.35)
                    lin = max(min_lin, speed_base * max(0.45, 1.0 - (abs(err) * 1.8)))
                    if ratio >= _digit_stop_box_ratio:
                        lin *= 0.55
                        msg = f"approaching_close:digit#{target_id} d={dist_m:.2f}m"
                    else:
                        msg = f"tracking:digit#{target_id} d={dist_m:.2f}m"
                ok, safety = publish_cmd_vel(lin, ang, record=False)
                with _digit_lock:
                    _digit_state = "arrived" if arrived else "track"
                    _digit_error_count = max(0, int(_digit_error_count) - 1)
                    _digit_last_cmd_ang = float(ang)
                    _digit_last_seen_ts = time.time()
                    _digit_last_msg = (safety if safety and safety != "OK" else msg) if ok else (safety or "digit_move_failed")
            else:
                with _digit_lock:
                    known = _digit_map.get(target_id)
                known_age = (now - float(known.get("seen_ts", 0.0))) if isinstance(known, dict) else 999.0
                target_age = (now - target_started_ts) if target_started_ts > 0.0 else 999.0
                # Simplified behavior for stability:
                # when target is not visible, do not drive forward; rotate only.
                search_sign = float(_digit_search_dir)
                search_reason = "scan"
                if isinstance(known, dict) and known_age <= max(1.0, float(_digit_map_ttl)):
                    target_bearing = float(known.get("bearing", 0.0))
                    yaw_err = _norm_angle(target_bearing - _digit_yaw_est)
                    if abs(yaw_err) > float(_digit_heading_deadband):
                        search_sign = 1.0 if yaw_err > 0.0 else -1.0
                        search_reason = "map_turn"
                elif detections:
                    ids_seen = sorted(int(d.get("id")) for d in detections if isinstance(d, dict) and "id" in d)
                    if ids_seen:
                        if target_id > max(ids_seen):
                            search_sign = 1.0
                            search_reason = "order_right"
                        elif target_id < min(ids_seen):
                            search_sign = -1.0
                            search_reason = "order_left"
                if (now - float(_digit_search_flip_ts)) >= max(0.5, float(_digit_search_flip_sec)):
                    _digit_search_dir *= -1.0
                    _digit_search_flip_ts = now
                    if search_reason == "scan":
                        search_sign = float(_digit_search_dir)
                with _digit_lock:
                    age = (time.time() - _digit_last_seen_ts) if _digit_last_seen_ts else 999.0
                if age > _digit_lost_timeout:
                    ang_cmd = abs(float(_digit_search_ang)) * (1.0 if search_sign >= 0.0 else -1.0)
                    ok, safety = publish_cmd_vel(0.0, ang_cmd, record=False)
                    with _digit_lock:
                        _digit_state = "scan"
                        _digit_last_cmd_ang = float(ang_cmd)
                        ordered_pairs = []
                        for mid, info in _digit_map.items():
                            ordered_pairs.append((float(info.get("bearing", 0.0)), int(mid)))
                        ordered_pairs.sort(key=lambda x: x[0])
                        ordered_ids = [mid for _, mid in ordered_pairs]
                        if target_age < 2.5:
                            s = f"scanning_digits:{ordered_ids}:{search_reason}"
                        else:
                            s = f"searching_digit:{ordered_ids}:{search_reason}"
                        _digit_last_msg = (safety if safety and safety != "OK" else s) if ok else (safety or "search_failed")
                else:
                    with _digit_lock:
                        _digit_state = "scan"
                        _digit_last_cmd_ang = 0.0
                        _digit_last_msg = "holding_digit"
        except Exception as exc:
            with _digit_lock:
                _digit_error_count += 1
                _digit_state = "scan"
                _digit_last_msg = f"digit_error:{exc}"
                if _digit_error_count >= int(_digit_error_disable_threshold):
                    _digit_enabled = False
                    _digit_state = "idle"
                    _digit_last_msg = f"digit_disabled:error_storm:{_digit_error_count}"
            publish_cmd_vel(0.0, 0.0, record=False)

        elapsed = time.time() - loop_ts
        time.sleep(max(0.0, interval - elapsed))


def _number_make_templates(cv2):
    fonts = [
        cv2.FONT_HERSHEY_SIMPLEX,
        cv2.FONT_HERSHEY_DUPLEX,
        cv2.FONT_HERSHEY_COMPLEX,
        cv2.FONT_HERSHEY_TRIPLEX,
    ]
    scales = [1.0, 1.2, 1.4, 1.7, 2.0]
    thicknesses = [2, 3, 4, 5]
    templates = {1: [], 2: [], 3: [], 4: []}
    hu_templates = {1: [], 2: [], 3: [], 4: []}
    for digit in (1, 2, 3, 4):
        txt = str(digit)
        for f in fonts:
            for sc in scales:
                for th in thicknesses:
                    canvas = 255 * np.ones((64, 64), dtype=np.uint8)
                    (tw, thh), _ = cv2.getTextSize(txt, f, sc, th)
                    x = max(0, (64 - tw) // 2)
                    y = max(thh + 2, (64 + thh) // 2)
                    cv2.putText(canvas, txt, (x, y), f, sc, (0,), th, cv2.LINE_AA)
                    _, bin_inv = cv2.threshold(canvas, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                    tmpl = cv2.resize(bin_inv, (32, 32), interpolation=cv2.INTER_AREA)
                    templates[digit].append(tmpl)
                    hu = _number_hu_signature(cv2, tmpl)
                    if hu is not None:
                        hu_templates[digit].append(hu)
    return {"pix": templates, "hu": hu_templates}


def _number_hu_signature(cv2, bin_img):
    if bin_img is None or bin_img.size == 0:
        return None
    b = bin_img
    if b.dtype != np.uint8:
        b = b.astype(np.uint8)
    if b.max() <= 1:
        b = (b * 255).astype(np.uint8)
    contours, _ = cv2.findContours(b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    m = cv2.moments(c)
    if m.get("m00", 0.0) <= 1e-6:
        return None
    hu = cv2.HuMoments(m).flatten()
    # Log-scale Hu moments for better numeric stability.
    hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-12)
    return hu.astype(np.float32)


def _number_classify_digit(cv2, roi_bin, templates):
    if roi_bin is None or roi_bin.size == 0:
        return None, 0.0, 1.0
    z = cv2.resize(roi_bin, (32, 32), interpolation=cv2.INTER_AREA)
    if z.max() > 1:
        _, z = cv2.threshold(z, 127, 255, cv2.THRESH_BINARY)
    zf = z.astype(np.float32) / 255.0
    pix_templates = templates.get("pix", {}) if isinstance(templates, dict) else templates
    hu_templates = templates.get("hu", {}) if isinstance(templates, dict) else {}
    hu_z = _number_hu_signature(cv2, z)
    nz = np.column_stack(np.where(z > 0))
    asp = 0.5
    if nz.size > 0:
        y_min = int(np.min(nz[:, 0]))
        y_max = int(np.max(nz[:, 0]))
        x_min = int(np.min(nz[:, 1]))
        x_max = int(np.max(nz[:, 1]))
        bw = max(1, x_max - x_min + 1)
        bh = max(1, y_max - y_min + 1)
        asp = float(bw) / float(bh)
    best_cls = None
    best_score = 1e9
    second = 1e9
    for k, arr in pix_templates.items():
        hu_arr = hu_templates.get(k, [])
        for t in arr:
            tf = t.astype(np.float32) / 255.0
            s_pix = float(np.mean(np.abs(zf - tf)))
            s_hu = 0.45
            if hu_z is not None and hu_arr:
                s_hu = min(float(np.mean(np.abs(hu_z - h))) for h in hu_arr)
                s_hu = min(1.0, s_hu / 6.0)
            s = (0.74 * s_pix) + (0.26 * s_hu)
            # Aspect-ratio priors to reduce 1<->2/3/4 confusion on big printed digits.
            if asp < 0.34:
                if int(k) == 1:
                    s *= 0.82
                else:
                    s *= 1.10
            elif asp > 0.70:
                if int(k) == 1:
                    s *= 1.20
                else:
                    s *= 0.93
            if s < best_score:
                second = best_score
                best_score = s
                best_cls = int(k)
            elif s < second:
                second = s
    if best_cls is None:
        return None, 0.0, 1.0
    conf = 0.0
    if second < 1e8:
        conf = max(0.0, min(1.0, (second - best_score) / max(1e-6, second)))
    return best_cls, conf, best_score


def number_status():
    with _number_lock:
        now = time.time()
        known_ids = []
        ordered_pairs = []
        for mid, info in _number_map.items():
            ts = float(info.get("seen_ts", 0.0)) if isinstance(info, dict) else 0.0
            if ts > 0.0 and (now - ts) <= max(0.5, float(_number_map_ttl)):
                known_ids.append(int(mid))
                ordered_pairs.append((float(info.get("bearing", 0.0)), int(mid)))
        known_ids.sort()
        ordered_pairs.sort(key=lambda x: x[0])
        ordered_ids_lr = [mid for _, mid in ordered_pairs]
        return {
            "enabled": bool(_number_enabled),
            "state": str(_number_state),
            "target_id": int(_number_target_id),
            "last_msg": str(_number_last_msg),
            "last_seen_age": (time.time() - _number_last_seen_ts) if _number_last_seen_ts else None,
            "count": len(_number_results),
            "size": dict(_number_size) if isinstance(_number_size, dict) else None,
            "known_ids": known_ids,
            "ordered_ids_lr": ordered_ids_lr,
        }


def number_stop(reason: str = "user"):
    global _number_enabled, _number_state, _number_last_msg, _number_last_cmd_ang, _number_arrive_streak
    global _number_target_last_err, _number_target_last_ratio, _number_target_last_seen_ts
    with _number_lock:
        was = bool(_number_enabled)
        _number_enabled = False
        _number_state = "idle"
        _number_last_cmd_ang = 0.0
        _number_arrive_streak = 0
        _number_target_last_err = 0.0
        _number_target_last_ratio = 0.0
        _number_target_last_seen_ts = 0.0
        _number_last_msg = f"stopped:{reason}" if reason else "stopped"
    publish_cmd_vel(0.0, 0.0, record=False)
    if was:
        return True, "번호 이동을 중지했습니다."
    return True, "번호 이동이 이미 중지 상태입니다."


def number_start(target_id: int):
    global _number_enabled, _number_state, _number_target_id, _number_last_msg, _number_thread
    global _number_search_flip_ts, _number_search_dir, _number_arrive_streak
    global _number_target_last_err, _number_target_last_ratio, _number_target_last_seen_ts
    if target_id not in {1, 2, 3, 4}:
        return False, "number id must be 1~4"
    now = time.time()
    with _number_lock:
        _number_enabled = True
        _number_state = "scan"
        _number_target_id = int(target_id)
        _number_last_msg = "started"
        _number_search_flip_ts = now
        _number_search_dir = 1.0
        _number_arrive_streak = 0
        _number_target_last_err = 0.0
        _number_target_last_ratio = 0.0
        _number_target_last_seen_ts = 0.0
    if _number_thread is None or not _number_thread.is_alive():
        _number_thread = threading.Thread(target=_number_worker, daemon=True)
        _number_thread.start()
    return True, f"번호 {target_id}로 이동을 시작합니다."


def _number_worker():
    global _number_enabled, _number_state, _number_last_msg, _number_last_seen_ts, _number_results, _number_size, _number_last_ts
    global _number_map, _number_yaw_est, _number_last_cmd_ang, _number_last_loop_ts
    global _number_search_dir, _number_search_flip_ts, _number_arrive_streak
    global _number_target_last_err, _number_target_last_ratio, _number_target_last_seen_ts
    interval = 1.0 / max(1.0, float(_number_cycle_hz))
    loop_idx = 0
    ocr_reader = None
    ocr_enabled = bool(_number_ocr_enabled)
    try:
        import cv2
        templates = _number_make_templates(cv2)
        if ocr_enabled:
            try:
                import easyocr
                # Use CPU mode and recognizer-only path for lower memory on Raspberry Pi.
                ocr_reader = easyocr.Reader(["en"], gpu=False, detector=False, recognizer=True, verbose=False)
            except Exception as exc:
                ocr_enabled = False
                with _number_lock:
                    _number_last_msg = f"number_ocr_disabled:{exc}"
    except Exception as exc:
        with _number_lock:
            _number_enabled = False
            _number_state = "idle"
            _number_last_msg = f"number_init_failed:{exc}"
        return

    while True:
        loop_idx += 1
        loop_ts = time.time()
        with _number_lock:
            enabled = bool(_number_enabled)
            target_id = int(_number_target_id)
            prev_loop_ts = float(_number_last_loop_ts or 0.0)
            prev_cmd_ang = float(_number_last_cmd_ang)
            prev_target_err = float(_number_target_last_err)
            prev_target_seen_ts = float(_number_target_last_seen_ts or 0.0)
        if not enabled:
            time.sleep(0.1)
            continue

        now = time.time()
        dt = (now - prev_loop_ts) if prev_loop_ts > 0.0 else 0.0
        with _number_lock:
            if dt > 0.0:
                _number_yaw_est = _norm_angle(_number_yaw_est + (prev_cmd_ang * dt))
            _number_last_loop_ts = now

        with _frame_lock:
            frame = _latest_frame.copy() if _latest_frame is not None else None
        if frame is None:
            time.sleep(0.1)
            continue

        try:
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, bw_white = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(bw_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections = []
            target = None
            target_candidates = []
            now = time.time()
            fov_rad = math.radians(max(20.0, min(140.0, float(_number_cam_fov_deg))))

            for c in contours:
                area = float(cv2.contourArea(c))
                if area < (w * h * 0.03) or area > (w * h * 0.9):
                    continue
                x, y, ww, hh = cv2.boundingRect(c)
                if ww < 18 or hh < 18:
                    continue
                ar = float(ww) / float(max(1, hh))
                if ar < 0.35 or ar > 1.9:
                    continue
                pad_x = int(ww * 0.08)
                pad_y = int(hh * 0.08)
                x0 = max(0, x + pad_x)
                y0 = max(0, y + pad_y)
                x1 = min(w, x + ww - pad_x)
                y1 = min(h, y + hh - pad_y)
                if x1 - x0 < 10 or y1 - y0 < 10:
                    continue
                roi = gray[y0:y1, x0:x1]
                roi_h, roi_w = roi.shape[:2]
                _, roi_inv = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                inner_contours, _ = cv2.findContours(roi_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not inner_contours:
                    continue
                # Detect each digit-like contour separately so target center is not the panel midpoint.
                roi_area = max(1.0, float((x1 - x0) * (y1 - y0)))
                min_digit_area = (roi_area * 0.025)
                inner_sorted = sorted(inner_contours, key=cv2.contourArea, reverse=True)
                for dc in inner_sorted[:8]:
                    da = float(cv2.contourArea(dc))
                    if da < min_digit_area:
                        continue
                    if da > (roi_area * 0.58):
                        # Reject panel border-like huge contours.
                        continue
                    dx, dy, dww, dhh = cv2.boundingRect(dc)
                    if dww < 8 or dhh < 12:
                        continue
                    # Reject contours touching ROI edge (panel frame / split line artifacts).
                    if dx <= 1 or dy <= 1 or (dx + dww) >= (roi_w - 1) or (dy + dhh) >= (roi_h - 1):
                        continue
                    dar = float(dww) / float(max(1, dhh))
                    if dar < 0.12 or dar > 1.6:
                        continue
                    extent = da / max(1.0, float(dww * dhh))
                    if extent < 0.14 or extent > 0.92:
                        continue
                    digit_roi = roi_inv[dy:dy+dhh, dx:dx+dww]
                    cls, conf, score = _number_classify_digit(cv2, digit_roi, templates)
                    # OCR fallback on candidate ROI only (not full-frame) for robustness/memory balance.
                    do_ocr = (loop_idx % max(1, int(_number_ocr_stride))) == 0
                    if do_ocr and ocr_enabled and ocr_reader is not None and (cls is None or score > 0.50):
                        try:
                            ocr_res = ocr_reader.readtext(digit_roi, detail=1, paragraph=False, allowlist="1234")
                            if ocr_res:
                                o_best = max(ocr_res, key=lambda r: float(r[2]) if len(r) > 2 else 0.0)
                                txt = str(o_best[1]).strip() if len(o_best) > 1 else ""
                                c = float(o_best[2]) if len(o_best) > 2 else 0.0
                                if txt and txt[0] in {"1", "2", "3", "4"} and c >= float(_number_ocr_threshold):
                                    cls = int(txt[0])
                                    conf = max(float(conf), c)
                                    score = min(float(score), 1.0 - c)
                        except Exception:
                            pass
                    if cls is None or score > 0.62:
                        continue
                    fx1 = float(x0 + dx)
                    fy1 = float(y0 + dy)
                    fx2 = float(x0 + dx + dww)
                    fy2 = float(y0 + dy + dhh)
                    ratio = ((fx2 - fx1) * (fy2 - fy1)) / max(1.0, float(w * h))
                    det = {
                        "id": int(cls),
                        "box": [fx1, fy1, fx2, fy2],
                        "ratio": float(ratio),
                        "conf": float(conf),
                        "score": float(score),
                    }
                    # De-duplicate same-class overlapping detections.
                    duplicate = False
                    for prev in detections:
                        if int(prev.get("id", -1)) != int(cls):
                            continue
                        px1, py1, px2, py2 = [float(v) for v in prev["box"]]
                        ix1 = max(px1, fx1)
                        iy1 = max(py1, fy1)
                        ix2 = min(px2, fx2)
                        iy2 = min(py2, fy2)
                        iw = max(0.0, ix2 - ix1)
                        ih = max(0.0, iy2 - iy1)
                        inter = iw * ih
                        pa = max(1.0, (px2 - px1) * (py2 - py1))
                        ca = max(1.0, (fx2 - fx1) * (fy2 - fy1))
                        if (inter / min(pa, ca)) > 0.5:
                            duplicate = True
                            if float(score) < float(prev.get("score", 9.9)):
                                prev.update(det)
                            break
                    if duplicate:
                        continue
                    detections.append(det)
                    cx = (fx1 + fx2) * 0.5
                    err = (cx / max(1.0, float(w))) - 0.5
                    bearing = _norm_angle(_number_yaw_est - (err * fov_rad))
                    with _number_lock:
                        _number_map[int(cls)] = {"bearing": float(bearing), "seen_ts": now, "ratio": float(ratio)}
                    if int(cls) == target_id:
                        target_candidates.append(det)

            if target_candidates:
                recent_target = (now - prev_target_seen_ts) <= max(0.2, float(_number_target_lock_sec))
                if recent_target:
                    def _cand_cost(d):
                        dcx = (float(d["box"][0]) + float(d["box"][2])) * 0.5
                        derr = (dcx / max(1.0, float(w))) - 0.5
                        # Keep the previously tracked target unless a clearly better area appears.
                        return abs(derr - prev_target_err) - (0.18 * float(d.get("ratio", 0.0)))
                    target = min(target_candidates, key=_cand_cost)
                else:
                    target = max(target_candidates, key=lambda d: float(d.get("ratio", 0.0)))

            with _number_lock:
                stale = [mid for mid, info in _number_map.items() if (now - float(info.get("seen_ts", 0.0))) > max(0.5, float(_number_map_ttl))]
                for mid in stale:
                    _number_map.pop(mid, None)
                _number_results = detections
                _number_size = {"w": int(w), "h": int(h)}
                _number_last_ts = time.time()

            if target is not None:
                cx = (target["box"][0] + target["box"][2]) * 0.5
                err = (cx / max(1.0, float(w))) - 0.5
                if (now - prev_target_seen_ts) <= max(0.2, float(_number_target_lock_sec)):
                    err = (prev_target_err * 0.62) + (err * 0.38)
                if abs(err) < float(_number_center_deadband):
                    err = 0.0
                raw_ang = _clamp(-err * _number_turn_gain, -_number_turn_max, _number_turn_max)
                # Smooth turn command to reduce left-right oscillation and overshoot.
                ang = _clamp((prev_cmd_ang * 0.45) + (raw_ang * 0.55), -_number_turn_max, _number_turn_max)
                ratio = float(target.get("ratio", 0.0))
                arrive_cond = (ratio >= float(_number_arrive_ratio) and abs(err) <= float(_number_arrive_center_err))
                if arrive_cond:
                    _number_arrive_streak += 1
                else:
                    _number_arrive_streak = 0
                if _number_arrive_streak >= int(_number_arrive_streak_need):
                    lin = 0.0
                    state = "arrived"
                    msg = f"arrived:number#{target_id} ratio={ratio:.3f}"
                elif abs(err) > 0.42:
                    lin = 0.0
                    state = "track"
                    msg = f"aligning:number#{target_id} ratio={ratio:.3f}"
                else:
                    forward_scale = max(0.45, 1.0 - (abs(err) * 1.4))
                    # If target looks still far away, push slightly stronger approach.
                    if ratio < 0.03:
                        forward_scale *= 1.20
                    lin = max(0.06, min(0.28, float(_number_forward_speed) * forward_scale))
                    state = "track"
                    msg = f"tracking:number#{target_id} ratio={ratio:.3f}"
                ok, safety = publish_cmd_vel(lin, ang, record=False)
                with _number_lock:
                    _number_state = state
                    _number_last_cmd_ang = float(ang)
                    _number_last_seen_ts = time.time()
                    _number_target_last_seen_ts = time.time()
                    _number_target_last_err = float(err)
                    _number_target_last_ratio = float(ratio)
                    _number_last_msg = (safety if safety and safety != "OK" else msg) if ok else (safety or "number_move_failed")
            else:
                _number_arrive_streak = 0
                recent_target = (now - prev_target_seen_ts) <= max(0.2, float(_number_target_lock_sec))
                if recent_target:
                    hold_ang = _clamp(-prev_target_err * float(_number_turn_gain), -0.35, 0.35)
                    hold_lin = float(_number_hold_forward_speed)
                    ok, safety = publish_cmd_vel(hold_lin, hold_ang, record=False)
                    with _number_lock:
                        _number_state = "track"
                        _number_last_cmd_ang = float(hold_ang)
                        _number_last_msg = (safety if safety and safety != "OK" else f"track_hold:number#{target_id}") if ok else (safety or "number_hold_failed")
                    elapsed = time.time() - loop_ts
                    time.sleep(max(0.0, interval - elapsed))
                    continue
                with _number_lock:
                    known = _number_map.get(target_id)
                search_sign = float(_number_search_dir)
                reason = "scan"
                if isinstance(known, dict):
                    yaw_err = _norm_angle(float(known.get("bearing", 0.0)) - _number_yaw_est)
                    if abs(yaw_err) > 0.08:
                        search_sign = 1.0 if yaw_err > 0.0 else -1.0
                        reason = "map_turn"
                    else:
                        ang_cmd = _clamp(yaw_err * float(_number_turn_gain), -0.22, 0.22)
                        lin_cmd = float(_number_map_seek_forward_speed)
                        ok, safety = publish_cmd_vel(lin_cmd, ang_cmd, record=False)
                        with _number_lock:
                            _number_state = "track"
                            _number_last_cmd_ang = float(ang_cmd)
                            _number_last_msg = (safety if safety and safety != "OK" else f"map_seek:number#{target_id}") if ok else (safety or "number_map_seek_failed")
                        elapsed = time.time() - loop_ts
                        time.sleep(max(0.0, interval - elapsed))
                        continue
                if (now - float(_number_search_flip_ts)) >= max(0.5, float(_number_search_flip_sec)):
                    _number_search_dir *= -1.0
                    _number_search_flip_ts = now
                    if reason == "scan":
                        search_sign = float(_number_search_dir)
                ang_cmd = abs(float(_number_search_ang)) * (1.0 if search_sign >= 0.0 else -1.0)
                ok, safety = publish_cmd_vel(0.0, ang_cmd, record=False)
                with _number_lock:
                    _number_state = "scan"
                    _number_last_cmd_ang = float(ang_cmd)
                    ordered = sorted([(float(v.get("bearing", 0.0)), int(k)) for k, v in _number_map.items()], key=lambda x: x[0])
                    ordered_ids = [k for _, k in ordered]
                    msg = f"scanning_numbers:{ordered_ids}:{reason}"
                    _number_last_msg = (safety if safety and safety != "OK" else msg) if ok else (safety or "number_search_failed")
        except Exception as exc:
            with _number_lock:
                _number_state = "scan"
                _number_last_msg = f"number_error:{exc}"
            publish_cmd_vel(0.0, 0.0, record=False)

        elapsed = time.time() - loop_ts
        time.sleep(max(0.0, interval - elapsed))


def cliff_guard_status():
    return {
        "enabled": bool(_cliff_guard_enabled),
        "mode": str(_cliff_guard_mode),
        "threshold": float(_cliff_ir_threshold),
        "sensor_age": (time.time() - _ir_last_ts) if _ir_last_ts else None,
        "ir": list(_ir_ranges) if isinstance(_ir_ranges, list) else None,
    }


def cliff_guard_start(mode: str = "low", threshold: float | None = None):
    global _cliff_guard_enabled, _cliff_guard_mode, _cliff_ir_threshold
    m = str(mode or "low").strip().lower()
    if m not in {"low", "high"}:
        return False, "mode must be low or high"
    _cliff_guard_mode = m
    if threshold is not None:
        try:
            _cliff_ir_threshold = max(1.0, min(4095.0, float(threshold)))
        except Exception:
            pass
    _cliff_guard_enabled = True
    return True, f"cliff guard ON (mode={_cliff_guard_mode}, threshold={_cliff_ir_threshold:.1f})"


def cliff_guard_stop():
    global _cliff_guard_enabled
    _cliff_guard_enabled = False
    return True, "cliff guard OFF"


def _cliff_guard_reason():
    if not _cliff_guard_enabled:
        return ""
    now = time.time()
    if _ir_last_ts <= 0.0 or (now - _ir_last_ts) > max(0.2, float(_cliff_sensor_timeout)):
        return "cliff_sensor_timeout"
    if not isinstance(_ir_ranges, list) or not _ir_ranges:
        return "cliff_ir_missing"
    vals = []
    for v in _ir_ranges[:3]:
        try:
            vals.append(float(v))
        except Exception:
            continue
    if not vals:
        return "cliff_ir_invalid"
    thr = float(_cliff_ir_threshold)
    if _cliff_guard_mode == "high":
        if any(v >= thr for v in vals):
            return f"cliff_ir_high={','.join(f'{v:.0f}' for v in vals)}"
    else:
        if any(v <= thr for v in vals):
            return f"cliff_ir_low={','.join(f'{v:.0f}' for v in vals)}"
    return ""


def _safety_adjust_linear(lin: float):
    if lin == 0.0:
        return 0.0, ""

    now = time.time()
    requested = float(lin)
    mag = abs(requested)
    cliff_reason = _cliff_guard_reason()
    if cliff_reason:
        return 0.0, f"SAFE_STOP({cliff_reason})"

    # Forward: use ultrasonic + front LiDAR.
    if requested > 0.0:
        us_raw = _us_range if (now - _us_last_ts) <= _safety_sensor_timeout else None
        us = us_raw if (us_raw is not None and us_raw >= float(_safety_us_min_valid)) else None
        scan = _scan_min_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None

        if us is None and scan is None:
            return 0.0, "SAFE_STOP(sensor_timeout)"

        # Forward behavior: prefer slowdown over abrupt stop.
        # Hard stop only in emergency distance.
        us_emg = (us is not None and us <= float(_safety_us_emergency_stop))
        scan_emg = (scan is not None and scan <= float(_safety_scan_emergency_stop))
        if us_emg or scan_emg:
            return 0.0, f"SAFE_STOP(emg us={us if us is not None else -1:.2f},scan={scan if scan is not None else -1:.2f})"

        limit = mag
        slowed = False

        if scan is not None and scan < _safety_scan_slow:
            span = max(1e-3, _safety_scan_slow - _safety_scan_stop)
            ratio = max(0.0, min(1.0, (scan - _safety_scan_stop) / span))
            scan_limit = _safety_min_speed + (limit - _safety_min_speed) * ratio
            if scan_limit < limit:
                slowed = True
                limit = scan_limit

        # Ultrasonic-only slowdown is noisy on some floors.
        # Use ultrasonic slowdown only when scan is unavailable, or when scan is also in the slow zone.
        if us is not None and us < _safety_us_slow:
            us_should_limit = (scan is None) or (scan < _safety_scan_slow)
            if us_should_limit:
                span = max(1e-3, _safety_us_slow - _safety_us_stop)
                ratio = max(0.0, min(1.0, (us - _safety_us_stop) / span))
                us_limit = _safety_min_speed + (limit - _safety_min_speed) * ratio
                if us_limit < limit:
                    slowed = True
                    limit = us_limit

        # Near-stop zone (but non-emergency): keep crawling instead of abrupt stop.
        near_stop = False
        if (us is not None and us <= _safety_us_stop) or (scan is not None and scan <= _safety_scan_stop):
            near_stop = True
        if near_stop:
            slowed = True
            limit = min(limit, max(float(_safety_min_speed), 0.05))

        safe_lin = max(0.0, min(mag, limit))
        if slowed and safe_lin < mag:
            return safe_lin, f"SAFE_SLOW(us={us if us is not None else -1:.2f},scan={scan if scan is not None else -1:.2f})"
        return safe_lin, ""

    # Backward: use rear LiDAR only.
    scan_back = _scan_rear_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
    if scan_back is not None and scan_back <= float(_safety_scan_back_emergency_stop):
        return 0.0, f"SAFE_STOP(emg scan_back={scan_back:.2f})"

    limit = mag
    slowed = False
    if scan_back is not None and scan_back < _safety_scan_back_slow:
        span = max(1e-3, _safety_scan_back_slow - _safety_scan_back_stop)
        ratio = max(0.0, min(1.0, (scan_back - _safety_scan_back_stop) / span))
        scan_limit = _safety_min_speed + (limit - _safety_min_speed) * ratio
        if scan_limit < limit:
            slowed = True
            limit = scan_limit
    if scan_back is not None and scan_back <= _safety_scan_back_stop:
        slowed = True
        limit = min(limit, max(float(_safety_min_speed), 0.05))

    safe_mag = max(0.0, min(mag, limit))
    safe_lin = -safe_mag
    if slowed and safe_mag < mag:
        return safe_lin, f"SAFE_SLOW(scan_back={scan_back if scan_back is not None else -1:.2f})"
    return safe_lin, ""


def publish_cmd_vel(lin, ang, record=True, manual_override=False):
    global _reverse_align_until, _last_req_lin, _autonav_enabled, _autonav_last_msg
    global _safety_avoid_until, _safety_avoid_back_until, _safety_avoid_dir, _safety_avoid_last_ts, _safety_avoid_resume_until
    if not _ros_enabled or _ros_pub is None:
        return False, "ROS2 not available"
    try:
        from geometry_msgs.msg import Twist
    except Exception:
        return False, "Twist message unavailable"
    req_lin = float(lin)
    req_ang = float(ang)

    # Rear single-caster platforms are typically unstable in reverse.
    # Damping reverse linear/angular commands reduces caster flutter.
    if req_lin < 0.0:
        req_lin *= max(0.2, min(1.0, float(_reverse_speed_scale)))
        req_ang *= max(0.2, min(1.0, float(_reverse_turn_scale)))
        now = time.time()
        with _cmd_state_lock:
            # On reverse entry, keep a short low-speed no-turn phase so
            # the caster can swivel and settle before full reverse torque.
            if _last_req_lin >= -1e-4:
                _reverse_align_until = now + float(_reverse_align_time)
            in_align = now < _reverse_align_until
            _last_req_lin = req_lin
        if in_align:
            req_lin = -min(abs(req_lin), max(0.03, float(_reverse_align_speed)))
            req_ang = 0.0
    else:
        with _cmd_state_lock:
            if req_lin > 1e-4:
                _reverse_align_until = 0.0
            _last_req_lin = req_lin

    now = time.time()
    if manual_override:
        # Human direct control is always top priority on /cmd_vel:
        # bypass auto-avoid/safety shaping and immediately apply requested twist.
        out_lin = float(req_lin)
        out_ang = float(req_ang)
        safety_msg = "MANUAL_OVERRIDE"
        with _safety_avoid_lock:
            _safety_avoid_until = 0.0
            _safety_avoid_back_until = 0.0
            _safety_avoid_resume_until = 0.0
    else:
        safe_lin, safety_msg = _safety_adjust_linear(req_lin)
        out_lin = float(safe_lin)
        out_ang = float(req_ang)

    if _safety_avoid_enabled and not manual_override:
        with _safety_avoid_lock:
            if now < float(_safety_avoid_until):
                if now < float(_safety_avoid_back_until):
                    out_lin = -abs(float(_safety_avoid_back_speed))
                    out_ang = 0.0
                    safety_msg = "SAFE_AVOID(backoff)"
                else:
                    out_lin = 0.0
                    out_ang = float(_safety_avoid_turn_speed) * float(_safety_avoid_dir)
                    safety_msg = "SAFE_AVOID(turn)"
            elif now < float(_safety_avoid_resume_until):
                # Short escape-forward window after a successful avoidance turn.
                esc_lin, esc_msg = _safety_adjust_linear(abs(float(_safety_avoid_resume_speed)))
                if esc_lin > 1e-3:
                    out_lin = float(esc_lin)
                    out_ang = 0.0
                    safety_msg = "SAFE_AVOID(resume)"
                else:
                    out_lin = 0.0
                    out_ang = float(_safety_avoid_turn_speed) * float(_safety_avoid_dir)
                    safety_msg = esc_msg or "SAFE_AVOID(resume_blocked)"
            elif (
                req_lin > 0.0
                and isinstance(safety_msg, str)
                and safety_msg.startswith("SAFE_STOP(")
                and ("us=" in safety_msg or "scan=" in safety_msg)
                and (now - float(_safety_avoid_last_ts)) >= float(_safety_avoid_cooldown)
            ):
                # User-requested behavior: rotate left in 30-degree steps on each collision.
                # Rotate to the left in fixed steps on each collision.
                _safety_avoid_dir = -1.0
                step_rad = math.radians(max(1.0, float(_safety_avoid_step_deg)))
                turn_span = step_rad / max(0.10, abs(float(_safety_avoid_turn_speed)))
                _safety_avoid_back_until = now + float(_safety_avoid_back_sec)
                _safety_avoid_until = float(_safety_avoid_back_until) + max(0.05, float(turn_span))
                _safety_avoid_resume_until = 0.0
                _safety_avoid_last_ts = now
                out_lin = -abs(float(_safety_avoid_back_speed))
                out_ang = 0.0
                safety_msg = f"{safety_msg}|SAFE_AVOID(step_left={float(_safety_avoid_step_deg):.1f}deg)"
            elif req_lin > 0.0 and float(_safety_avoid_until) > 0.0:
                # Avoidance turn ended. If front is clear now, force short straight escape.
                scan = _scan_min_range if (now - _scan_last_ts) <= _safety_sensor_timeout else None
                us = _us_range if (now - _us_last_ts) <= _safety_sensor_timeout else None
                scan_clear = (scan is not None and scan >= float(_safety_avoid_clear_scan))
                us_clear = (us is not None and us >= float(_safety_avoid_clear_us))
                # Resume only when front is clearly open; avoid premature forward push.
                front_clear = ((scan is not None and us is not None and scan_clear and us_clear) or
                               (scan is not None and us is None and scan_clear) or
                               (us is not None and scan is None and us_clear))
                if front_clear:
                    _safety_avoid_resume_until = now + float(_safety_avoid_resume_sec)
                    esc_lin, esc_msg = _safety_adjust_linear(abs(float(_safety_avoid_resume_speed)))
                    out_lin = float(esc_lin)
                    out_ang = 0.0
                    safety_msg = (esc_msg if esc_msg and esc_msg != "OK" else "SAFE_AVOID(resume_start)")
                else:
                    out_lin = 0.0
                    out_ang = float(_safety_avoid_turn_speed) * float(_safety_avoid_dir)
                    safety_msg = "SAFE_AVOID(seek_clear)"

    if record:
        with _autonav_lock:
            if _autonav_enabled:
                _autonav_enabled = False
                _autonav_last_msg = "stopped:manual_override"
        _nav_record_cmd(now, out_lin, out_ang)
    msg = Twist()
    msg.linear.x = float(out_lin)
    msg.angular.z = float(out_ang)
    try:
        _ros_pub.publish(msg)
    except Exception as exc:
        return False, f"ROS publish failed: {exc}"
    if safety_msg:
        return True, safety_msg
    return True, "OK"


def publish_lcd_name(name: str):
    global _ros_name_last, _ros_name_last_ts
    if not _ros_enabled or _ros_name_pub is None:
        return
    text = (name or "")
    if not text.strip() and _lcd_default_text:
        text = _lcd_default_text
    now = time.time()
    if text == _ros_name_last and (now - _ros_name_last_ts) < 0.5:
        return
    try:
        from std_msgs.msg import String
    except Exception:
        return
    msg = String()
    msg.data = text
    try:
        _ros_name_pub.publish(msg)
    except Exception:
        return
    _ros_name_last = text
    _ros_name_last_ts = now


def _battery_cb(msg):
    global _battery_voltage
    _battery_voltage = float(getattr(msg, "data", None)) if msg is not None else None


def _us_cb(msg):
    global _us_range, _us_last_ts
    try:
        _us_range = float(getattr(msg, "range", None))
        _us_last_ts = time.time()
    except Exception:
        _us_range = None


def _ir_cb(msg):
    global _ir_ranges, _ir_last_ts
    try:
        _ir_ranges = list(getattr(msg, "data", []))
        _ir_last_ts = time.time()
    except Exception:
        _ir_ranges = None


def _odom_cb(msg):
    global _odom_xy, _odom_yaw, _odom_last_ts
    try:
        pose = getattr(msg, "pose", None)
        pose_pose = getattr(pose, "pose", None) if pose is not None else None
        pos = getattr(pose_pose, "position", None) if pose_pose is not None else None
        ori = getattr(pose_pose, "orientation", None) if pose_pose is not None else None
        if pos is None:
            return
        x = float(getattr(pos, "x", 0.0) or 0.0)
        y = float(getattr(pos, "y", 0.0) or 0.0)
        yaw = None
        if ori is not None:
            qx = float(getattr(ori, "x", 0.0) or 0.0)
            qy = float(getattr(ori, "y", 0.0) or 0.0)
            qz = float(getattr(ori, "z", 0.0) or 0.0)
            qw = float(getattr(ori, "w", 1.0) or 1.0)
            siny = 2.0 * ((qw * qz) + (qx * qy))
            cosy = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
            yaw = math.atan2(siny, cosy)
        with _odom_lock:
            _odom_xy = (x, y)
            _odom_yaw = yaw
            _odom_last_ts = time.time()
    except Exception:
        return


def _scan_cb(msg):
    global _scan_min_range, _scan_left_range, _scan_right_range, _scan_rear_range, _scan_last_ts
    try:
        ranges = list(getattr(msg, "ranges", []) or [])
        angle_min = float(getattr(msg, "angle_min", 0.0) or 0.0)
        angle_inc = float(getattr(msg, "angle_increment", 0.0) or 0.0)
        half = math.radians(max(5.0, min(120.0, float(_safety_scan_front_half_deg))))
        center = math.radians(float(_safety_scan_center_deg))
        side_half = math.radians(35.0)
        rear_center = center + math.pi
        left_center = center + (math.pi / 2.0)
        right_center = center - (math.pi / 2.0)
        min_valid = max(0.03, float(_safety_scan_min_valid))

        all_valid = []
        front_valid = []
        left_valid = []
        right_valid = []
        rear_valid = []
        for i, rv in enumerate(ranges):
            if rv is None:
                continue
            r = float(rv)
            if not math.isfinite(r) or r < min_valid:
                continue
            all_valid.append(r)
            if angle_inc != 0.0:
                ang = angle_min + (i * angle_inc)
                # Shortest signed angular distance from configured front center.
                d = math.atan2(math.sin(ang - center), math.cos(ang - center))
                if abs(d) <= half:
                    front_valid.append(r)
                dl = math.atan2(math.sin(ang - left_center), math.cos(ang - left_center))
                if abs(dl) <= side_half:
                    left_valid.append(r)
                dr = math.atan2(math.sin(ang - right_center), math.cos(ang - right_center))
                if abs(dr) <= side_half:
                    right_valid.append(r)
                db = math.atan2(math.sin(ang - rear_center), math.cos(ang - rear_center))
                if abs(db) <= half:
                    rear_valid.append(r)

        valid = front_valid if front_valid else all_valid
        if valid:
            front_min = min(front_valid) if front_valid else min(valid)
            rear_min = min(rear_valid) if rear_valid else None
            left_min = min(left_valid) if left_valid else None
            right_min = min(right_valid) if right_valid else None
            if bool(_safety_scan_swap_lr):
                left_min, right_min = right_min, left_min
            if bool(_safety_scan_swap_fb):
                front_min, rear_min = rear_min, front_min
            _scan_min_range = front_min
            _scan_left_range = left_min
            _scan_right_range = right_min
            _scan_rear_range = rear_min
            _scan_last_ts = time.time()
        else:
            _scan_min_range = None
            _scan_left_range = None
            _scan_right_range = None
            _scan_rear_range = None
    except Exception:
        _scan_min_range = None
        _scan_left_range = None
        _scan_right_range = None
        _scan_rear_range = None


def _track_fps():
    global _fps, _fps_frames, _fps_last_ts
    now = time.time()
    if _fps_last_ts == 0.0:
        _fps_last_ts = now
    _fps_frames += 1
    dt = now - _fps_last_ts
    if dt >= 1.0:
        _fps = _fps_frames / dt
        _fps_frames = 0
        _fps_last_ts = now


def _frame_worker(fps: float):
    global _latest_frame, _latest_frame_ts, _latest_frame_err
    _latest_frame_err = "frame_thread_started"
    _latest_frame_ts = time.time()
    try:
        import cv2
        import numpy as np
    except Exception as exc:
        _latest_frame_err = f"deps_missing: {exc}"
        _latest_frame_ts = time.time()
        logging.warning("Frame deps missing: %s", exc)
        return

    interval = 1.0 / max(0.1, fps)
    while True:
        start = time.time()
        try:
            frame = output.frame if output else None
            if not frame:
                time.sleep(0.05)
                continue
            arr = np.frombuffer(frame, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                time.sleep(0.05)
                continue
            with _frame_lock:
                _latest_frame = img
                _latest_frame_ts = time.time()
                _latest_frame_err = None
        except Exception as exc:
            _latest_frame_err = f"frame_error: {exc}"
            _latest_frame_ts = time.time()
            logging.warning("Frame error: %s", exc)
        finally:
            elapsed = time.time() - start
            time.sleep(max(0.0, interval - elapsed))


def _vision_worker(model_path: str, fps: float, imgsz: int):
    global _vision_pose, _vision_err, _vision_last_ts
    _vision_err = "vision_thread_started"
    _vision_last_ts = time.time()
    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
    except Exception as exc:
        _vision_err = f"deps_missing: {exc}"
        _vision_last_ts = time.time()
        logging.warning("Vision deps missing: %s", exc)
        return

    if not model_path or not os.path.exists(model_path):
        _vision_err = f"model_not_found: {model_path}"
        _vision_last_ts = time.time()
        logging.warning("YOLO model not found: %s", model_path)
        return

    try:
        model = YOLO(model_path)
    except Exception as exc:
        _vision_err = f"yolo_init_failed: {exc}"
        _vision_last_ts = time.time()
        logging.warning("YOLO init failed: %s", exc)
        return

    interval = 1.0 / max(0.1, fps)
    while True:
        start = time.time()
        try:
            with _autonav_lock:
                nav_busy = bool(_autonav_enabled)
            if nav_busy:
                # Prioritize pose/autonav responsiveness over secondary model work.
                time.sleep(max(interval, 0.25))
                continue
            with _frame_lock:
                img = _latest_frame.copy() if _latest_frame is not None else None
            if img is None:
                time.sleep(0.1)
                continue
            h, w = img.shape[:2]

            results = model(
                img,
                verbose=False,
                imgsz=max(128, int(imgsz)),
                max_det=5,
            )
            res = results[0]

            boxes = []
            if res.boxes is not None and len(res.boxes) > 0:
                xyxy = res.boxes.xyxy.cpu().numpy()
                conf = res.boxes.conf.cpu().numpy() if res.boxes.conf is not None else None
                for i, b in enumerate(xyxy[:5]):
                    c = float(conf[i]) if conf is not None else 0.0
                    boxes.append([float(b[0]), float(b[1]), float(b[2]), float(b[3]), c])

            kps = []
            if hasattr(res, "keypoints") and res.keypoints is not None:
                xy = res.keypoints.xy.cpu().numpy()
                for person in xy[:5]:
                    pts = [[float(p[0]), float(p[1])] for p in person]
                    kps.append(pts)

            with _vision_lock:
                _vision_pose = {"w": w, "h": h, "boxes": boxes, "kps": kps}
                _vision_err = None
                _vision_last_ts = time.time()
        except Exception as exc:
            _vision_err = f"vision_error: {exc}"
            _vision_last_ts = time.time()
            logging.warning("Vision error: %s", exc)
        finally:
            elapsed = time.time() - start
            time.sleep(max(0.0, interval - elapsed))


def _object_worker(model_path: str, fps: float, imgsz: int, max_det: int):
    global _obj_results, _obj_size, _obj_err, _obj_last_ts
    _obj_err = "object_thread_started"
    _obj_last_ts = time.time()
    try:
        from ultralytics import YOLO
    except Exception as exc:
        _obj_err = f"deps_missing: {exc}"
        _obj_last_ts = time.time()
        logging.warning("Object deps missing: %s", exc)
        return

    if not model_path or not os.path.exists(model_path):
        _obj_err = f"model_not_found: {model_path}"
        _obj_last_ts = time.time()
        logging.warning("Object model not found: %s", model_path)
        return

    try:
        model = YOLO(model_path)
    except Exception as exc:
        _obj_err = f"obj_init_failed: {exc}"
        _obj_last_ts = time.time()
        logging.warning("Object model init failed: %s", exc)
        return

    interval = 1.0 / max(0.1, fps)
    while True:
        start = time.time()
        try:
            with _frame_lock:
                img = _latest_frame.copy() if _latest_frame is not None else None
            if img is None:
                time.sleep(0.1)
                continue

            h, w = img.shape[:2]
            results = model(
                img,
                verbose=False,
                imgsz=max(128, int(imgsz)),
                max_det=max(1, int(max_det)),
            )
            res = results[0]
            names = getattr(res, "names", {}) or {}
            out = []
            person_dets = []

            if res.boxes is not None and len(res.boxes) > 0:
                xyxy = res.boxes.xyxy.cpu().numpy()
                conf = res.boxes.conf.cpu().numpy() if res.boxes.conf is not None else []
                cls = res.boxes.cls.cpu().numpy() if res.boxes.cls is not None else []
                for i, b in enumerate(xyxy):
                    c = float(conf[i]) if i < len(conf) else 0.0
                    cid = int(cls[i]) if i < len(cls) else -1
                    label = str(names.get(cid, cid))
                    out.append(
                        {
                            "box": [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
                            "label": label,
                            "score": c,
                        }
                    )
                    if label.lower() == "person":
                        person_dets.append(
                            {
                                "box": [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
                                "score": c,
                            }
                        )

            with _obj_lock:
                _obj_results = out
                _obj_size = {"w": w, "h": h}
                _obj_err = None
                _obj_last_ts = time.time()
            _update_person_tracks(person_dets, w, h)
        except Exception as exc:
            _obj_err = f"obj_error: {exc}"
            _obj_last_ts = time.time()
            logging.warning("Object error: %s", exc)
        finally:
            elapsed = time.time() - start
            time.sleep(max(0.0, interval - elapsed))


def _load_face_db(path: str):
    global _face_db
    if not path:
        _face_db = []
        return
    if not os.path.exists(path):
        _face_db = []
        return
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            _face_db = data
        else:
            _face_db = []
    except Exception:
        _face_db = []


def _save_face_db(path: str):
    if not path:
        return
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(_face_db, f, ensure_ascii=False)
    os.replace(tmp, path)


def _cosine_sim(a, b):
    import numpy as np
    a = np.asarray(a, dtype=np.float32)
    b = np.asarray(b, dtype=np.float32)
    na = np.linalg.norm(a) + 1e-6
    nb = np.linalg.norm(b) + 1e-6
    return float(np.dot(a, b) / (na * nb))


def _face_worker(model_name: str, fps: float, db_path: str, threshold: float, det_size: int):
    global _face_app, _face_err, _face_last_ts, _face_results, _face_db, _face_temp_db, _face_size
    _face_err = "face_thread_started"
    _face_last_ts = time.time()
    try:
        import cv2
        from insightface.app import FaceAnalysis
    except Exception as exc:
        _face_err = f"deps_missing: {exc}"
        _face_last_ts = time.time()
        logging.warning("Face deps missing: %s", exc)
        return

    try:
        _face_app = FaceAnalysis(name=model_name, providers=["CPUExecutionProvider"])
        ds = max(128, int(det_size))
        _face_app.prepare(ctx_id=0, det_size=(ds, ds))
    except Exception as exc:
        _face_err = f"face_init_failed: {exc}"
        _face_last_ts = time.time()
        logging.warning("Face init failed: %s", exc)
        return

    _load_face_db(db_path)

    interval = 1.0 / max(0.1, fps)
    while True:
        start = time.time()
        try:
            with _frame_lock:
                img = _latest_frame.copy() if _latest_frame is not None else None
            if img is None:
                time.sleep(0.1)
                continue

            h, w = img.shape[:2]
            with _face_lock:
                faces = _face_app.get(img) if _face_app else []

            results = []
            best_name = ""
            best_score = 0.0
            if faces:
                for f in faces[:5]:
                    name = "unknown"
                    score = 0.0
                    db_all = _face_temp_db + _face_db
                    if getattr(f, "normed_embedding", None) is not None and db_all:
                        emb = f.normed_embedding.tolist()
                        db_best_name = "unknown"
                        db_best_score = 0.0
                        for item in db_all:
                            db_emb = item.get("embedding")
                            if not db_emb:
                                continue
                            s = _cosine_sim(emb, db_emb)
                            if s > db_best_score:
                                db_best_score = s
                                db_best_name = item.get("name") or "unknown"
                        if db_best_score >= threshold:
                            name = db_best_name
                            score = db_best_score
                    box = [float(v) for v in f.bbox.tolist()]
                    results.append({"box": box, "name": name, "score": score})
                    if name != "unknown" and score >= threshold and score > best_score:
                        best_name = name
                        best_score = score

            _face_results = results
            _face_size = {"w": w, "h": h}
            _face_err = None
            _face_last_ts = time.time()
            if best_name:
                if _mark_first_seen_today(best_name):
                    publish_lcd_name(f"오늘 처음이네요.\n{best_name} 님")
                else:
                    publish_lcd_name(f"{best_name} 님 반가워요.")
            elif faces:
                publish_lcd_name("안녕하세요.")
            else:
                publish_lcd_name("")
        except Exception as exc:
            _face_err = f"face_error: {exc}"
            _face_last_ts = time.time()
            logging.warning("Face error: %s", exc)
        finally:
            elapsed = time.time() - start
            time.sleep(max(0.0, interval - elapsed))


def register_face(name: str, persist: bool = True):
    global _face_db, _face_temp_db
    if not _face_app:
        return False, "Face app not ready"
    if not name:
        return False, "Name required"
    try:
        import cv2
        import numpy as np
    except Exception as exc:
        return False, f"deps_missing: {exc}"
    with _frame_lock:
        img = _latest_frame.copy() if _latest_frame is not None else None
    if img is None:
        return False, "No camera frame"
    with _face_lock:
        faces = _face_app.get(img) if _face_app else []
    if not faces:
        return False, "No face detected"
    # choose largest face
    faces.sort(key=lambda f: float((f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1])), reverse=True)
    f = faces[0]
    if getattr(f, "normed_embedding", None) is None:
        return False, "Embedding unavailable"
    emb = f.normed_embedding.tolist()
    if persist:
        _face_db = [item for item in _face_db if item.get("name") != name]
        _face_db.append({"name": name, "embedding": emb})
        _save_face_db(_face_db_path)
    else:
        _face_temp_db = [item for item in _face_temp_db if item.get("name") != name]
        _face_temp_db.append({"name": name, "embedding": emb})
    return True, f"Face registered: {name}"


def delete_face(name: str):
    global _face_db, _face_temp_db
    if not name:
        return False, "Name required"
    before_persistent = len(_face_db)
    before_temp = len(_face_temp_db)
    _face_db = [item for item in _face_db if item.get("name") != name]
    _face_temp_db = [item for item in _face_temp_db if item.get("name") != name]
    if len(_face_db) == before_persistent and len(_face_temp_db) == before_temp:
        return False, "Name not found"
    if len(_face_db) != before_persistent:
        _save_face_db(_face_db_path)
    return True, f"Face deleted: {name}"


def clear_temp_faces():
    global _face_temp_db
    _face_temp_db = []
    return True, "Temporary face data cleared"


def _today_key() -> str:
    return datetime.now().strftime("%Y-%m-%d")


def _load_greet_state(path: str):
    global _greet_day_key, _greeted_today
    with _greet_lock:
        _greet_day_key = _today_key()
        _greeted_today = {}
        if not path or not os.path.exists(path):
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return
            day = str(data.get("day", "") or "")
            names = data.get("names", {})
            if day != _greet_day_key or not isinstance(names, dict):
                return
            _greeted_today = {str(k): bool(v) for k, v in names.items() if str(k).strip()}
        except Exception:
            _greeted_today = {}


def _save_greet_state(path: str):
    if not path:
        return
    with _greet_lock:
        payload = {
            "day": _greet_day_key or _today_key(),
            "names": _greeted_today,
        }
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False)
    os.replace(tmp, path)


def _mark_first_seen_today(name: str) -> bool:
    global _greet_day_key, _greeted_today, _greet_pending
    n = str(name or "").strip()
    if not n:
        return False
    first_today = False
    with _greet_lock:
        today = _today_key()
        if _greet_day_key != today:
            _greet_day_key = today
            _greeted_today = {}
        if not _greeted_today.get(n):
            _greeted_today[n] = True
            _greet_pending = n
            first_today = True
    if first_today and _greet_state_path:
        _save_greet_state(_greet_state_path)
    return first_today


def _consume_greet_pending() -> str:
    global _greet_pending
    with _greet_lock:
        name = _greet_pending
        _greet_pending = ""
    return name


def _openai_chat(messages):
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        return False, "OPENAI_API_KEY not set"
    model = os.environ.get("OPENAI_CHAT_MODEL", "gpt-4o-mini").strip() or "gpt-4o-mini"
    body = {
        "model": model,
        "messages": messages,
        "temperature": 0.3,
        "max_tokens": 84,
    }
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        "https://api.openai.com/v1/chat/completions",
        data=data,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            raw = resp.read().decode("utf-8")
        payload = json.loads(raw)
        choices = payload.get("choices") or []
        if not choices:
            return False, "No response choices"
        message = choices[0].get("message") or {}
        text = str(message.get("content") or "").strip()
        if not text:
            return False, "Empty response"
        return True, text
    except urllib.error.HTTPError as exc:
        try:
            detail = exc.read().decode("utf-8")
        except Exception:
            detail = str(exc)
        return False, f"OpenAI HTTP {exc.code}: {detail}"
    except Exception as exc:
        return False, f"OpenAI request failed: {exc}"


def chat_reply(user_text: str):
    if _is_plate_number_query(user_text):
        num = str(_plate_number_text or "11").strip() or "11"
        return True, f"제 번호판 번호는 {num}번입니다."

    system_prompt = (
        "너는 Pinky 로봇의 한국어 음성 도우미다. "
        "짧고 명확하게 1~2문장으로 답하고, 과도한 장식 없이 실용적으로 말하라. "
        "로봇 주행 명령은 사용자가 '핑키'로 시작해 별도로 처리하므로, 일반 대화에 집중하라."
    )
    with _chat_lock:
        messages = [{"role": "system", "content": system_prompt}]
        if _chat_history:
            messages.extend(_chat_history[-_chat_history_max:])
        messages.append({"role": "user", "content": user_text})
    ok, reply = _openai_chat(messages)
    if not ok:
        return False, reply
    with _chat_lock:
        _chat_history.append({"role": "user", "content": user_text})
        _chat_history.append({"role": "assistant", "content": reply})
        if len(_chat_history) > _chat_history_max:
            _chat_history[:] = _chat_history[-_chat_history_max:]
    return True, reply


def _server_scene_summary_text():
    parts = []
    with _face_lock:
        faces = list(_face_results) if _face_results is not None else []
    with _obj_lock:
        objs = list(_obj_results) if _obj_results is not None else []
    if faces:
        known = [f.get("name") for f in faces if f.get("name") and f.get("name") != "unknown"]
        if known:
            uniq = list(dict.fromkeys(known))
            if len(uniq) == 1:
                parts.append(f"{uniq[0]}님이 보입니다")
            else:
                parts.append(f"{', '.join(uniq[:2])}님이 보입니다")
        else:
            parts.append(f"사람 {len(faces)}명이 보입니다")
    else:
        parts.append("사람은 보이지 않습니다")

    counts = {}
    for o in objs:
        label = str(o.get("label", "")).lower()
        if not label or label == "person":
            continue
        counts[label] = counts.get(label, 0) + 1
    if counts:
        top = sorted(counts.items(), key=lambda x: x[1], reverse=True)[:3]
        parts.append("사물: " + ", ".join([f"{k} {v}개" for k, v in top]))

    if isinstance(_us_range, (int, float)):
        parts.append(f"초음파 {float(_us_range):.2f}m")
    if isinstance(_battery_voltage, (int, float)):
        parts.append(f"배터리 {float(_battery_voltage):.2f}V")
    return ". ".join(parts)


def _strip_json_fence(text: str):
    t = (text or "").strip()
    if t.startswith("```"):
        t = re.sub(r"^```(?:json)?", "", t).strip()
        t = re.sub(r"```$", "", t).strip()
    return t


def _openai_intent(user_text: str, scene_hint: str = "", recognized_name: str = ""):
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        return False, "OPENAI_API_KEY not set"
    model = os.environ.get("OPENAI_CHAT_MODEL", "gpt-4o-mini").strip() or "gpt-4o-mini"
    system_prompt = (
        "너는 Pinky 로봇의 명령 라우터다. "
        "사용자 발화를 아래 intent 중 하나로 분류하고 JSON만 출력한다. "
        "허용 intent: stop, move_forward, move_backward, turn_left, turn_right, follow_me, "
        "auto_nav_start, auto_nav_stop, save_home, return_home, cancel_return, describe_scene, "
        "park_reverse, park_slot, chat, unknown. "
        "park_slot인 경우 args.slot(1~10 정수)을 반드시 넣어라. "
        "반드시 형식: {\"intent\":\"...\",\"reply\":\"...\",\"confidence\":0.0,\"args\":{}} "
        "reply는 1문장으로 짧게. JSON 외 텍스트 금지."
    )
    user_payload = {
        "text": user_text,
        "recognized_name": recognized_name or "",
        "scene_hint": scene_hint or "",
    }
    body = {
        "model": model,
        "messages": [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": json.dumps(user_payload, ensure_ascii=False)},
        ],
        "temperature": 0.1,
        "max_tokens": 180,
    }
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        "https://api.openai.com/v1/chat/completions",
        data=data,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            raw = resp.read().decode("utf-8")
        payload = json.loads(raw)
        choices = payload.get("choices") or []
        if not choices:
            return False, "No response choices"
        msg = choices[0].get("message") or {}
        content = _strip_json_fence(str(msg.get("content") or ""))
        out = json.loads(content)
        if not isinstance(out, dict):
            return False, "Intent payload not object"
        return True, out
    except Exception as exc:
        return False, f"Intent parse failed: {exc}"


def _local_intent_fallback(text: str):
    t = re.sub(r"\s+", "", text or "").lower()
    if any(k in t for k in ["주차", "파킹", "parking"]):
        m = re.search(r"(10|[1-9])번", t)
        if not m:
            m = re.search(r"(?:slot|슬롯|주차면)(10|[1-9])", t)
        if not m:
            m = re.search(r"(10|[1-9])", t)
        if m:
            slot = int(m.group(1))
            slot = max(1, min(10, slot))
            return {
                "intent": "park_slot",
                "reply": f"{slot}번 주차면으로 이동합니다.",
                "confidence": 0.9,
                "args": {"slot": slot},
            }
    if any(k in t for k in ["정지", "멈춰", "스톱", "stop"]):
        return {"intent": "stop", "reply": "정지 명령을 수행합니다.", "confidence": 0.9, "args": {}}
    if any(k in t for k in ["자율주행중지", "자율주행정지", "오토중지", "자동주행중지", "autonavstop"]):
        return {"intent": "auto_nav_stop", "reply": "자율주행을 중지합니다.", "confidence": 0.92, "args": {}}
    if any(k in t for k in ["자율주행", "자율주행시작", "오토주행", "자동주행", "오토파일럿", "autonav"]):
        return {"intent": "auto_nav_start", "reply": "자율주행을 시작합니다.", "confidence": 0.9, "args": {}}
    if any(k in t for k in ["따라와", "followme"]):
        return {"intent": "follow_me", "reply": "따라가기 모드를 시작합니다.", "confidence": 0.9, "args": {}}
    if any(k in t for k in ["출발점저장", "출발지저장", "홈저장", "집저장"]):
        return {"intent": "save_home", "reply": "출발점을 저장합니다.", "confidence": 0.9, "args": {}}
    if any(k in t for k in ["돌아와", "복귀", "집으로가"]):
        return {"intent": "return_home", "reply": "복귀를 시작합니다.", "confidence": 0.85, "args": {}}
    if any(k in t for k in ["복귀중지", "돌아오기중지"]):
        return {"intent": "cancel_return", "reply": "복귀를 중지합니다.", "confidence": 0.85, "args": {}}
    if any(k in t for k in ["앞", "전진", "forward", "go"]):
        return {"intent": "move_forward", "reply": "앞으로 이동합니다.", "confidence": 0.8, "args": {}}
    if any(k in t for k in ["뒤", "후진", "back"]):
        return {"intent": "move_backward", "reply": "뒤로 이동합니다.", "confidence": 0.8, "args": {}}
    if any(k in t for k in ["좌회전", "왼", "left"]):
        return {"intent": "turn_left", "reply": "좌회전합니다.", "confidence": 0.8, "args": {}}
    if any(k in t for k in ["우회전", "오", "right"]):
        return {"intent": "turn_right", "reply": "우회전합니다.", "confidence": 0.8, "args": {}}
    if any(k in t for k in ["뭐보여", "뭐가보여", "화면설명", "장면설명", "주변설명"]):
        return {"intent": "describe_scene", "reply": "", "confidence": 0.8, "args": {}}
    if any(k in t for k in ["주차", "후진주차"]):
        return {"intent": "park_reverse", "reply": "후진 주차 기능은 준비 중입니다.", "confidence": 0.7, "args": {}}
    return {"intent": "chat", "reply": "", "confidence": 0.4, "args": {}}


_ALLOWED_INTENTS = {
    "stop",
    "move_forward",
    "move_backward",
    "turn_left",
    "turn_right",
    "follow_me",
    "auto_nav_start",
    "auto_nav_stop",
    "save_home",
    "return_home",
    "cancel_return",
    "describe_scene",
    "park_reverse",
    "park_slot",
    "chat",
    "unknown",
}

_CONTROL_INTENTS = {
    "stop",
    "move_forward",
    "move_backward",
    "turn_left",
    "turn_right",
    "follow_me",
    "auto_nav_start",
    "auto_nav_stop",
    "save_home",
    "return_home",
    "cancel_return",
    "park_reverse",
    "park_slot",
}


def _contains_control_keywords(text: str) -> bool:
    t = re.sub(r"\s+", "", text or "").lower()
    keys = [
        "정지", "멈춰", "스톱", "stop",
        "자율주행", "자율주행시작", "자율주행중지", "자율주행정지", "오토주행", "오토중지", "자동주행", "autonav",
        "앞", "전진", "forward", "go",
        "뒤", "후진", "back",
        "좌회전", "왼", "left",
        "우회전", "오", "right",
        "따라와", "followme",
        "복귀", "돌아와", "집으로가",
        "출발점저장", "출발지저장", "홈저장", "집저장",
        "복귀중지", "돌아오기중지",
        "주차", "후진주차", "파킹", "parking",
    ]
    return any(k in t for k in keys)


def _is_plate_number_query(text: str) -> bool:
    t = re.sub(r"\s+", "", text or "").lower()
    if not t:
        return False
    has_plate_word = any(k in t for k in ["번호판", "차량번호", "번호"])
    has_query_word = any(k in t for k in ["몇", "뭐", "무슨", "알려", "말해", "다시", "번호"])
    has_id11 = ("id11" in t) or ("11번" in t)
    return (has_plate_word and has_query_word) or has_id11


def _sanitize_intent_payload(parsed: dict) -> dict:
    if not isinstance(parsed, dict):
        parsed = {}
    intent = str(parsed.get("intent", "unknown") or "unknown").strip().lower()
    if intent not in _ALLOWED_INTENTS:
        intent = "unknown"

    try:
        confidence = float(parsed.get("confidence", 0.0) or 0.0)
    except Exception:
        confidence = 0.0
    confidence = max(0.0, min(1.0, confidence))

    reply = str(parsed.get("reply", "") or "").strip()

    raw_args = parsed.get("args", {})
    if not isinstance(raw_args, dict):
        raw_args = {}
    args = {}
    if "speed_scale" in raw_args:
        try:
            args["speed_scale"] = max(0.2, min(1.5, float(raw_args.get("speed_scale"))))
        except Exception:
            pass
    if "duration_ms" in raw_args:
        try:
            args["duration_ms"] = int(max(80, min(4000, int(raw_args.get("duration_ms")))))
        except Exception:
            pass
    if "slot" in raw_args:
        try:
            args["slot"] = int(max(1, min(10, int(raw_args.get("slot")))))
        except Exception:
            pass

    return {
        "intent": intent,
        "reply": reply,
        "confidence": confidence,
        "args": args,
        "source": str(parsed.get("source", "unknown") or "unknown"),
    }


def assistant_decide(user_text: str, recognized_name: str = ""):
    if _is_plate_number_query(user_text):
        num = str(_plate_number_text or "11").strip() or "11"
        return True, {
            "intent": "chat",
            "reply": f"제 번호판 번호는 {num}번입니다.",
            "confidence": 1.0,
            "args": {},
            "source": "rule_plate_number",
            "execute": False,
        }

    scene_hint = _server_scene_summary_text()
    ok, parsed = _openai_intent(user_text, scene_hint=scene_hint, recognized_name=recognized_name)
    if not ok:
        parsed = _local_intent_fallback(user_text)
        parsed["source"] = "local_fallback_api"
    else:
        parsed["source"] = "openai"

    parsed = _sanitize_intent_payload(parsed)
    intent = parsed["intent"]
    reply = parsed["reply"]
    confidence = parsed["confidence"]
    args = parsed["args"]

    # Guardrail: control intents from GPT require minimum confidence.
    min_conf = max(0.0, min(1.0, float(os.environ.get("OPENAI_INTENT_MIN_CONF", "0.55") or 0.55)))
    if parsed.get("source") == "openai":
        if intent in _CONTROL_INTENTS and confidence < min_conf:
            fb = _local_intent_fallback(user_text)
            fb["source"] = "local_fallback_low_conf"
            parsed = _sanitize_intent_payload(fb)
            intent, reply, confidence, args = parsed["intent"], parsed["reply"], parsed["confidence"], parsed["args"]
        elif intent in {"chat", "unknown"} and _contains_control_keywords(user_text):
            fb = _local_intent_fallback(user_text)
            fb["source"] = "local_fallback_keyword"
            parsed = _sanitize_intent_payload(fb)
            intent, reply, confidence, args = parsed["intent"], parsed["reply"], parsed["confidence"], parsed["args"]

    if intent == "describe_scene":
        if not reply:
            reply = scene_hint or "현재 장면 정보를 확인할 수 없습니다."
    elif intent == "chat":
        ok_chat, rep = chat_reply(user_text)
        if ok_chat:
            reply = rep
        elif not reply:
            reply = rep
    elif intent == "park_reverse" and not reply:
        reply = "후진 주차 기능은 준비 중입니다."

    out = {
        "intent": intent,
        "reply": reply,
        "confidence": confidence,
        "args": args,
        "source": parsed.get("source", "unknown"),
        "execute": intent in _CONTROL_INTENTS,
    }
    return True, out

def _led_timer_cb():
    global _led_state, _led_last_ts, _led_pending_state, _led_pending_count
    if _led_mode == "off":
        return
    # No ultrasonic data yet
    if _us_range is None:
        return
    # Threshold scheme:
    # far: off
    # near: 2 green in center
    # closer: 4 yellow in center
    # very close: 8 red
    d = _us_range
    if d is None:
        return
    if d >= 0.8:
        count = 0
        color = (0, 0, 0)
        pixels = []
    elif d >= 0.5:
        count = 2
        color = (0, 255, 0)
        pixels = [3, 4]
    elif d >= 0.3:
        count = 4
        color = (255, 165, 0)
        pixels = [2, 3, 4, 5]
    else:
        count = 8
        color = (255, 0, 0)
        pixels = list(range(8))

    # Apply global brightness scaling (0.0~1.0) for softer LEDs.
    br = max(0.0, min(1.0, float(_led_brightness)))
    scaled_color = (
        int(color[0] * br),
        int(color[1] * br),
        int(color[2] * br),
    )

    state = (count, scaled_color)
    if state == _led_state:
        _led_pending_state = None
        _led_pending_count = 0
        return
    # Debounce LED state changes to avoid flicker near ultrasonic threshold boundaries.
    if state != _led_pending_state:
        _led_pending_state = state
        _led_pending_count = 1
        return
    _led_pending_count += 1
    if _led_pending_count < 3:
        return
    now = time.time()
    if _led_last_ts and (now - _led_last_ts) < _led_min_interval:
        return
    _led_pending_state = None
    _led_pending_count = 0
    _led_state = state
    _led_last_ts = now

    if not _led_client or not _led_client.service_is_ready():
        return

    try:
        from pinky_interfaces.srv import SetLed
    except Exception:
        return

    # Clear first to avoid leaving old colors on other pixels.
    clear_req = SetLed.Request()
    clear_req.command = "clear"
    clear_req.pixels = []
    clear_req.r = 0
    clear_req.g = 0
    clear_req.b = 0
    _led_client.call_async(clear_req)

    if count > 0:
        set_req = SetLed.Request()
        set_req.command = "set_pixel"
        set_req.pixels = pixels
        set_req.r, set_req.g, set_req.b = scaled_color
        _led_client.call_async(set_req)


def main():
    parser = argparse.ArgumentParser(description="Pinky MJPEG server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--led-mode", type=str, default="auto", choices=["auto", "off"])
    parser.add_argument("--led-min-interval", type=float, default=1.5)
    parser.add_argument("--led-brightness", type=float, default=0.12)
    parser.add_argument("--safety-us-stop", type=float, default=0.25)
    parser.add_argument("--safety-us-slow", type=float, default=0.45)
    parser.add_argument("--safety-us-min-valid", type=float, default=0.03)
    parser.add_argument("--safety-scan-stop", type=float, default=0.20)
    parser.add_argument("--safety-scan-slow", type=float, default=0.40)
    parser.add_argument("--safety-scan-back-stop", type=float, default=0.20)
    parser.add_argument("--safety-scan-back-slow", type=float, default=0.40)
    parser.add_argument("--safety-scan-min-valid", type=float, default=0.10)
    parser.add_argument("--safety-scan-front-half-deg", type=float, default=55.0)
    parser.add_argument("--safety-scan-center-deg", type=float, default=0.0)
    parser.add_argument("--safety-scan-swap-lr", type=int, default=1)
    parser.add_argument("--safety-scan-swap-fb", type=int, default=0)
    parser.add_argument("--safety-sensor-timeout", type=float, default=0.8)
    parser.add_argument("--safety-min-speed", type=float, default=0.08)
    parser.add_argument("--safety-avoid-enabled", type=int, default=1)
    parser.add_argument("--safety-avoid-back-sec", type=float, default=0.35)
    parser.add_argument("--safety-avoid-turn-sec", type=float, default=0.55)
    parser.add_argument("--safety-avoid-back-speed", type=float, default=0.08)
    parser.add_argument("--safety-avoid-turn-speed", type=float, default=0.55)
    parser.add_argument("--safety-avoid-step-deg", type=float, default=30.0)
    parser.add_argument("--safety-avoid-cooldown", type=float, default=1.0)
    parser.add_argument("--safety-avoid-resume-sec", type=float, default=0.60)
    parser.add_argument("--safety-avoid-resume-speed", type=float, default=0.08)
    parser.add_argument("--safety-avoid-clear-scan", type=float, default=0.24)
    parser.add_argument("--safety-avoid-clear-us", type=float, default=0.20)
    parser.add_argument("--cliff-guard", action="store_true")
    parser.add_argument("--cliff-mode", type=str, default="low", choices=["low", "high"])
    parser.add_argument("--cliff-ir-threshold", type=float, default=120.0)
    parser.add_argument("--cliff-sensor-timeout", type=float, default=0.8)
    parser.add_argument("--reverse-speed-scale", type=float, default=0.45)
    parser.add_argument("--reverse-turn-scale", type=float, default=0.60)
    parser.add_argument("--reverse-align-time", type=float, default=0.55)
    parser.add_argument("--reverse-align-speed", type=float, default=0.08)
    parser.add_argument("--hflip", action="store_true")
    parser.add_argument("--vflip", action="store_true")
    parser.add_argument("--camera-timeout", type=int, default=20)
    parser.add_argument("--pose-model", type=str, default="/home/pinky/pinky_visionNav/models/yolo11n-pose.pt")
    parser.add_argument("--pose-fps", type=float, default=0.0)
    parser.add_argument("--pose-imgsz", type=int, default=256)
    parser.add_argument("--obj-model", type=str, default="/home/pinky/pinky_visionNav/models/yolo11n.pt")
    parser.add_argument("--obj-fps", type=float, default=6.0)
    parser.add_argument("--obj-imgsz", type=int, default=256)
    parser.add_argument("--lcd-default-text", type=str, default="11")
    parser.add_argument("--obj-max-det", type=int, default=8)
    parser.add_argument("--person-min-score", type=float, default=0.22)
    parser.add_argument("--person-track-iou", type=float, default=0.24)
    parser.add_argument("--person-track-max-miss", type=int, default=7)
    parser.add_argument("--person-track-max-tracks", type=int, default=4)
    parser.add_argument("--person-track-smooth", type=float, default=0.60)
    parser.add_argument("--face-model", type=str, default="buffalo_s")
    parser.add_argument("--face-fps", type=float, default=0.5)
    parser.add_argument("--face-det-size", type=int, default=224)
    parser.add_argument("--decode-fps", type=float, default=12.0)
    parser.add_argument("--face-db", type=str, default="/home/pinky/pinky_web/face_db.json")
    parser.add_argument("--face-threshold", type=float, default=0.35)
    parser.add_argument("--greet-state", type=str, default="/home/pinky/pinky_web/greet_state.json")
    parser.add_argument("--digit-fps", type=float, default=5.0)
    parser.add_argument("--digit-turn-gain", type=float, default=2.0)
    parser.add_argument("--digit-forward-speed", type=float, default=0.18)
    parser.add_argument("--digit-stop-box-ratio", type=float, default=0.10)
    parser.add_argument("--digit-arrive-box-ratio", type=float, default=0.18)
    parser.add_argument("--digit-forward-min-scale", type=float, default=0.60)
    parser.add_argument("--digit-marker-size-m", type=float, default=0.16)
    parser.add_argument("--digit-arrive-dist-m", type=float, default=0.45)
    parser.add_argument("--digit-arrive-ratio-fallback", type=float, default=0.30)
    parser.add_argument("--digit-lost-timeout", type=float, default=1.0)
    parser.add_argument("--digit-switch-backoff-sec", type=float, default=0.8)
    parser.add_argument("--digit-backoff-speed", type=float, default=0.10)
    parser.add_argument("--digit-search-ang", type=float, default=0.35)
    parser.add_argument("--digit-cam-fov-deg", type=float, default=62.0)
    parser.add_argument("--digit-map-ttl", type=float, default=45.0)
    parser.add_argument("--digit-heading-gain", type=float, default=1.8)
    parser.add_argument("--digit-heading-deadband", type=float, default=0.14)
    parser.add_argument("--digit-seek-forward-speed", type=float, default=0.08)
    parser.add_argument("--number-fps", type=float, default=5.0)
    parser.add_argument("--number-turn-gain", type=float, default=2.0)
    parser.add_argument("--number-forward-speed", type=float, default=0.20)
    parser.add_argument("--number-arrive-ratio", type=float, default=0.30)
    parser.add_argument("--number-arrive-center-err", type=float, default=0.12)
    parser.add_argument("--number-arrive-streak-need", type=int, default=4)
    parser.add_argument("--number-search-ang", type=float, default=0.35)
    parser.add_argument("--number-lost-timeout", type=float, default=0.8)
    parser.add_argument("--number-map-ttl", type=float, default=20.0)
    parser.add_argument("--number-cam-fov-deg", type=float, default=62.0)
    parser.add_argument("--number-map-seek-forward-speed", type=float, default=0.10)
    parser.add_argument("--number-target-lock-sec", type=float, default=0.9)
    parser.add_argument("--number-center-deadband", type=float, default=0.08)
    parser.add_argument("--number-hold-forward-speed", type=float, default=0.07)
    parser.add_argument("--number-ocr-enabled", type=int, default=1)
    parser.add_argument("--number-ocr-threshold", type=float, default=0.40)
    parser.add_argument("--number-ocr-stride", type=int, default=2)
    args = parser.parse_args()
    if args.obj_model and not os.path.exists(args.obj_model):
        obj_fallbacks = []
        if args.pose_model:
            obj_fallbacks.append(args.pose_model)
        if args.obj_model.endswith(".pt"):
            obj_fallbacks.append(args.obj_model[:-3] + "-pose.pt")
        fallback = next((p for p in obj_fallbacks if p and os.path.exists(p)), None)
        if fallback:
            logging.warning(
                "Object model not found at %s; falling back to %s",
                args.obj_model,
                fallback,
            )
            args.obj_model = fallback
    if args.obj_model and args.pose_model:
        try:
            if os.path.abspath(args.obj_model) == os.path.abspath(args.pose_model):
                # Avoid running duplicate YOLO models at full rate on the same CPU.
                args.obj_fps = min(float(args.obj_fps), 0.3)
        except Exception:
            pass

    global PAGE
    # Clamp to camera max size 320x240
    if args.width > 320:
        args.width = 320
    if args.height > 240:
        args.height = 240
    PAGE = PAGE.replace("__RES__", f"{args.width}x{args.height}").replace("__FPS__", str(args.fps))

    global output
    output = StreamingOutput()

    picam2 = None
    last_err = None
    start = time.time()
    while time.time() - start < args.camera_timeout:
        try:
            picam2 = Picamera2()
            break
        except Exception as exc:
            last_err = exc
            logging.warning("Camera not ready, retrying: %s", exc)
            time.sleep(1)
    if picam2 is None:
        logging.error("Camera init failed after %ss: %s", args.camera_timeout, last_err)
        output = None
    else:
        transform = Transform(hflip=args.hflip, vflip=args.vflip)
        frame_us = int(1_000_000 / max(1, args.fps))
        # Camera node can be registered but stream-start may still fail transiently.
        # Retry start_recording instead of terminating the whole web process.
        start_rec_budget_sec = min(12.0, max(5.0, float(args.camera_timeout)))
        start_rec_deadline = time.time() + float(start_rec_budget_sec)
        start_rec_last_err = None
        while time.time() < start_rec_deadline:
            try:
                config = picam2.create_video_configuration(
                    # Use BGR888 so OpenCV processing and browser stream colors stay consistent.
                    main={"size": (args.width, args.height), "format": "BGR888"},
                    transform=transform,
                    controls={"FrameDurationLimits": (frame_us, frame_us)},
                )
                picam2.configure(config)
                picam2.start_recording(JpegEncoder(), FileOutput(output))
                start_rec_last_err = None
                break
            except Exception as exc:
                start_rec_last_err = exc
                logging.warning("Camera stream start failed, retrying: %s", exc)
                try:
                    picam2.stop_recording()
                except Exception:
                    pass
                try:
                    picam2.close()
                except Exception:
                    pass
                picam2 = None
                time.sleep(1.0)
                try:
                    picam2 = Picamera2()
                except Exception as init_exc:
                    start_rec_last_err = init_exc
                    logging.warning("Camera reopen failed, retrying: %s", init_exc)
                    time.sleep(1.0)
        if start_rec_last_err is not None:
            logging.error("Camera stream start failed after retries: %s", start_rec_last_err)
            picam2 = None
            output = None

    global _led_mode, _led_min_interval, _led_brightness, _lcd_default_text, _plate_number_text
    global _safety_us_stop, _safety_us_slow, _safety_us_min_valid, _safety_scan_stop, _safety_scan_slow
    global _safety_scan_back_stop, _safety_scan_back_slow
    global _safety_scan_min_valid, _safety_scan_front_half_deg, _safety_scan_center_deg, _safety_scan_swap_lr, _safety_scan_swap_fb
    global _safety_sensor_timeout, _safety_min_speed
    global _safety_avoid_enabled, _safety_avoid_back_sec, _safety_avoid_turn_sec
    global _safety_avoid_back_speed, _safety_avoid_turn_speed, _safety_avoid_step_deg, _safety_avoid_cooldown
    global _safety_avoid_resume_sec, _safety_avoid_resume_speed, _safety_avoid_clear_scan, _safety_avoid_clear_us
    global _cliff_guard_enabled, _cliff_guard_mode, _cliff_ir_threshold, _cliff_sensor_timeout
    global _reverse_speed_scale, _reverse_turn_scale, _reverse_align_time, _reverse_align_speed
    global _autonav_cycle_hz, _autonav_search_ang, _autonav_search_ang_slow, _autonav_lost_timeout
    global _autonav_target_max_age, _autonav_cmd_hold_timeout, _autonav_search_flip_period
    global _autonav_turn_max
    global _digit_cycle_hz, _digit_turn_gain, _digit_forward_speed, _digit_stop_box_ratio, _digit_arrive_box_ratio, _digit_forward_min_scale
    global _digit_marker_size_m, _digit_arrive_dist_m, _digit_arrive_ratio_fallback, _digit_lost_timeout
    global _digit_switch_backoff_sec, _digit_backoff_speed, _digit_search_ang
    global _digit_cam_fov_deg, _digit_map_ttl, _digit_heading_gain, _digit_heading_deadband, _digit_seek_forward_speed
    global _number_cycle_hz, _number_turn_gain, _number_forward_speed, _number_arrive_ratio
    global _number_arrive_center_err, _number_arrive_streak_need, _number_search_ang, _number_lost_timeout
    global _number_map_ttl, _number_cam_fov_deg, _number_map_seek_forward_speed
    global _number_target_lock_sec, _number_center_deadband, _number_hold_forward_speed
    global _number_ocr_enabled, _number_ocr_threshold, _number_ocr_stride
    global _person_min_score, _person_track_iou, _person_track_max_miss, _person_track_max_tracks, _person_track_smooth
    _led_mode = args.led_mode
    _led_min_interval = max(0.0, args.led_min_interval)
    _led_brightness = max(0.0, min(1.0, args.led_brightness))
    _safety_us_stop = max(0.05, float(args.safety_us_stop))
    _safety_us_slow = max(_safety_us_stop + 0.05, float(args.safety_us_slow))
    _safety_us_min_valid = max(0.0, min(0.20, float(args.safety_us_min_valid)))
    _safety_scan_stop = max(0.05, float(args.safety_scan_stop))
    _safety_scan_slow = max(_safety_scan_stop + 0.05, float(args.safety_scan_slow))
    _safety_scan_back_stop = max(0.05, float(args.safety_scan_back_stop))
    _safety_scan_back_slow = max(_safety_scan_back_stop + 0.05, float(args.safety_scan_back_slow))
    _safety_scan_min_valid = max(0.03, min(0.30, float(args.safety_scan_min_valid)))
    _safety_scan_front_half_deg = max(5.0, min(120.0, float(args.safety_scan_front_half_deg)))
    _safety_scan_center_deg = max(-180.0, min(180.0, float(args.safety_scan_center_deg)))
    _safety_scan_swap_lr = bool(int(args.safety_scan_swap_lr))
    _safety_scan_swap_fb = bool(int(args.safety_scan_swap_fb))
    _safety_sensor_timeout = max(0.2, float(args.safety_sensor_timeout))
    _safety_min_speed = max(0.03, min(0.2, float(args.safety_min_speed)))
    _safety_avoid_enabled = bool(int(args.safety_avoid_enabled))
    _safety_avoid_back_sec = max(0.0, min(2.0, float(args.safety_avoid_back_sec)))
    _safety_avoid_turn_sec = max(0.0, min(2.5, float(args.safety_avoid_turn_sec)))
    _safety_avoid_back_speed = max(0.03, min(0.25, float(args.safety_avoid_back_speed)))
    _safety_avoid_turn_speed = max(0.10, min(1.50, float(args.safety_avoid_turn_speed)))
    _safety_avoid_step_deg = max(5.0, min(120.0, float(args.safety_avoid_step_deg)))
    _safety_avoid_cooldown = max(0.0, min(5.0, float(args.safety_avoid_cooldown)))
    _safety_avoid_resume_sec = max(0.0, min(2.0, float(args.safety_avoid_resume_sec)))
    _safety_avoid_resume_speed = max(0.03, min(0.25, float(args.safety_avoid_resume_speed)))
    _safety_avoid_clear_scan = max(0.08, min(3.0, float(args.safety_avoid_clear_scan)))
    _safety_avoid_clear_us = max(0.08, min(3.0, float(args.safety_avoid_clear_us)))
    _cliff_guard_enabled = bool(args.cliff_guard)
    _cliff_guard_mode = str(args.cliff_mode or "low").strip().lower()
    _cliff_ir_threshold = max(1.0, min(4095.0, float(args.cliff_ir_threshold)))
    _cliff_sensor_timeout = max(0.2, float(args.cliff_sensor_timeout))
    _reverse_speed_scale = max(0.2, min(1.0, float(args.reverse_speed_scale)))
    _reverse_turn_scale = max(0.2, min(1.0, float(args.reverse_turn_scale)))
    _reverse_align_time = max(0.0, min(1.5, float(args.reverse_align_time)))
    _reverse_align_speed = max(0.03, min(0.2, float(args.reverse_align_speed)))
    pose_fps = max(0.1, float(args.pose_fps if args.pose_fps > 0.0 else args.obj_fps))
    _autonav_cycle_hz = max(6.0, min(12.0, pose_fps * 1.4))
    _autonav_target_max_age = max(0.35, min(1.1, 2.6 / pose_fps))
    _autonav_cmd_hold_timeout = max(0.20, min(0.60, _autonav_target_max_age * 0.65))
    _autonav_lost_timeout = max(1.0, _autonav_target_max_age + 0.55)
    _autonav_search_ang = 0.72
    _autonav_search_ang_slow = max(0.18, min(0.35, _autonav_search_ang * 0.42))
    _autonav_search_flip_period = 0.9
    _autonav_turn_max = 0.82
    _digit_cycle_hz = max(1.0, min(15.0, float(args.digit_fps)))
    _digit_turn_gain = max(0.2, min(5.0, float(args.digit_turn_gain)))
    _digit_forward_speed = max(0.03, min(0.6, float(args.digit_forward_speed)))
    _digit_stop_box_ratio = max(0.01, min(0.5, float(args.digit_stop_box_ratio)))
    _digit_arrive_box_ratio = max(_digit_stop_box_ratio + 0.01, min(0.8, float(args.digit_arrive_box_ratio)))
    _digit_forward_min_scale = max(0.2, min(1.0, float(args.digit_forward_min_scale)))
    _digit_marker_size_m = max(0.03, min(1.0, float(args.digit_marker_size_m)))
    _digit_arrive_dist_m = max(0.10, min(2.5, float(args.digit_arrive_dist_m)))
    _digit_arrive_ratio_fallback = max(0.05, min(0.9, float(args.digit_arrive_ratio_fallback)))
    _digit_lost_timeout = max(0.2, min(4.0, float(args.digit_lost_timeout)))
    _digit_switch_backoff_sec = max(0.0, min(3.0, float(args.digit_switch_backoff_sec)))
    _digit_backoff_speed = max(0.03, min(0.25, float(args.digit_backoff_speed)))
    _digit_search_ang = max(0.05, min(1.2, float(args.digit_search_ang)))
    _digit_cam_fov_deg = max(20.0, min(140.0, float(args.digit_cam_fov_deg)))
    _digit_map_ttl = max(2.0, min(180.0, float(args.digit_map_ttl)))
    _digit_heading_gain = max(0.2, min(4.0, float(args.digit_heading_gain)))
    _digit_heading_deadband = max(0.02, min(0.5, float(args.digit_heading_deadband)))
    _digit_seek_forward_speed = max(0.03, min(0.25, float(args.digit_seek_forward_speed)))
    _number_cycle_hz = max(1.0, min(15.0, float(args.number_fps)))
    _number_turn_gain = max(0.2, min(5.0, float(args.number_turn_gain)))
    _number_forward_speed = max(0.03, min(0.6, float(args.number_forward_speed)))
    _number_arrive_ratio = max(0.05, min(0.9, float(args.number_arrive_ratio)))
    _number_arrive_center_err = max(0.03, min(0.4, float(args.number_arrive_center_err)))
    _number_arrive_streak_need = max(1, min(12, int(args.number_arrive_streak_need)))
    _number_search_ang = max(0.05, min(1.2, float(args.number_search_ang)))
    _number_lost_timeout = max(0.2, min(4.0, float(args.number_lost_timeout)))
    _number_map_ttl = max(1.0, min(90.0, float(args.number_map_ttl)))
    _number_cam_fov_deg = max(20.0, min(140.0, float(args.number_cam_fov_deg)))
    _number_map_seek_forward_speed = max(0.03, min(0.25, float(args.number_map_seek_forward_speed)))
    _number_target_lock_sec = max(0.2, min(2.5, float(args.number_target_lock_sec)))
    _number_center_deadband = max(0.02, min(0.20, float(args.number_center_deadband)))
    _number_hold_forward_speed = max(0.03, min(0.20, float(args.number_hold_forward_speed)))
    _number_ocr_enabled = bool(int(args.number_ocr_enabled))
    _number_ocr_threshold = max(0.05, min(0.99, float(args.number_ocr_threshold)))
    _number_ocr_stride = max(1, min(12, int(args.number_ocr_stride)))
    _person_min_score = max(0.05, min(0.90, float(args.person_min_score)))
    _person_track_iou = max(0.05, min(0.85, float(args.person_track_iou)))
    _person_track_max_miss = max(1, min(25, int(args.person_track_max_miss)))
    _person_track_max_tracks = max(1, min(8, int(args.person_track_max_tracks)))
    _person_track_smooth = max(0.05, min(0.95, float(args.person_track_smooth)))
    _lcd_default_text = str(args.lcd_default_text or "").strip()
    _plate_number_text = _lcd_default_text or "11"
    logging.info(
        "Person tracker tuned: score>=%.2f iou=%.2f miss=%d smooth=%.2f tracks=%d",
        _person_min_score,
        _person_track_iou,
        _person_track_max_miss,
        _person_track_smooth,
        _person_track_max_tracks,
    )
    ros_ok = init_ros()
    if not ros_ok:
        logging.warning("ROS2 not available. Robot control/LCD publish will not work.")
    elif _lcd_default_text:
        publish_lcd_name(_lcd_default_text)

    threading.Thread(target=_frame_worker, args=(args.decode_fps,), daemon=True).start()
    if float(args.pose_fps) > 0.0:
        threading.Thread(
            target=_vision_worker,
            args=(args.pose_model, args.pose_fps, args.pose_imgsz),
            daemon=True,
        ).start()
    else:
        logging.info("Pose worker disabled (pose-fps <= 0). Using person detector + tracker for follow mode.")
    threading.Thread(
        target=_object_worker,
        args=(args.obj_model, args.obj_fps, args.obj_imgsz, args.obj_max_det),
        daemon=True,
    ).start()
    global _face_db_path, _face_threshold, _greet_state_path
    _face_db_path = args.face_db
    _face_threshold = args.face_threshold
    _greet_state_path = args.greet_state
    _load_greet_state(_greet_state_path)
    threading.Thread(
        target=_face_worker,
        args=(args.face_model, args.face_fps, args.face_db, args.face_threshold, args.face_det_size),
        daemon=True,
    ).start()

    server_address = (args.host, args.port)
    httpd = StreamingServer(server_address, StreamingHandler)
    print(f"MJPEG stream: http://{args.host}:{args.port}/ (client: http://<PINKY_IP>:{args.port}/)")
    try:
        httpd.serve_forever()
    finally:
        if picam2 is not None:
            picam2.stop_recording()
        shutdown_ros()


if __name__ == "__main__":
    main()
