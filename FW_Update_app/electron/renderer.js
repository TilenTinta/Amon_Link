const backendUrl = window.electronAPI.backendUrl;

const portSelect = document.getElementById("portSelect");
const baudSelect = document.getElementById("baudSelect");
const refreshPortsBtn = document.getElementById("refreshPorts");
const connectBtn = document.getElementById("connectBtn");
const statusBadge = document.getElementById("statusBadge");
const lastMessage = document.getElementById("lastMessage");
const fileInput = document.getElementById("fileInput");
const uploadBtn = document.getElementById("uploadBtn");
const startUploadBtn = document.getElementById("startUploadBtn");
const jumpAppBtn = document.getElementById("jumpAppBtn");
const progressFill = document.getElementById("progressFill");
const fileLabel = document.getElementById("fileLabel");
const logList = document.getElementById("logList");
const clearLogBtn = document.getElementById("clearLog");
const backendStatus = document.getElementById("backendStatus");
const crcValue = document.getElementById("crcValue");
const bootVer = document.getElementById("bootVer");
const deviceCrc32 = document.getElementById("deviceCrc32");
const crcMatch = document.getElementById("crcMatch");

let allowUpload = true;
let uploadWasInProgress = false;

const actionButtons = [
  refreshPortsBtn,
  connectBtn,
  uploadBtn,
  startUploadBtn,
  jumpAppBtn,
];

function addLog(message) {
  if (!message) {
    return;
  }
  const item = document.createElement("div");
  item.className = "log-item";
  const timestamp = new Date().toLocaleTimeString();
  item.textContent = `[${timestamp}] ${message}`;
  logList.prepend(item);
  if (logList.children.length > 80) {
    logList.removeChild(logList.lastChild);
  }
}

function setStatus(state) {
  statusBadge.textContent = state.connection_status || "Disconnected";
  const connected = state.connection_status?.includes("Connected");
  if (connected) {
    statusBadge.classList.add("connected");
  } else {
    statusBadge.classList.remove("connected");
  }
  lastMessage.textContent = state.last_message || "No messages yet";
  if (state.upload_name) {
    fileLabel.textContent = state.upload_name;
  } else {
    fileLabel.textContent = "No file selected";
  }
  progressFill.style.width = `${state.upload_progress || 0}%`;
  crcValue.textContent = state.upload_crc32 || "-";
  bootVer.textContent = state.device_boot_ver || "-";
  deviceCrc32.textContent = state.device_crc32 || "-";
  if (state.crc_match === "match") {
    crcMatch.textContent = "Match";
  } else if (state.crc_match === "mismatch") {
    crcMatch.textContent = "Mismatch";
  } else {
    crcMatch.textContent = "-";
  }
  connectBtn.textContent = connected ? "Disconnect" : "Connect";
  connectBtn.classList.toggle("danger", connected);
  connectBtn.classList.toggle("primary", !connected);

  if (uploadWasInProgress && !state.is_uploading && state.upload_progress === 100) {
    window.alert("Firmware update finished.");
  }
  uploadWasInProgress = Boolean(state.is_uploading);
}

async function fetchJson(path, options = {}) {
  const controller = new AbortController();
  const timeoutMs = options.timeoutMs ?? 8000;
  const timeoutId = setTimeout(() => controller.abort(), timeoutMs);
  const response = await fetch(`${backendUrl}${path}`, {
    ...options,
    signal: controller.signal,
  }).finally(() => clearTimeout(timeoutId));
  if (!response.ok) {
    const text = await response.text();
    throw new Error(text || response.statusText);
  }
  return response.json();
}

function setBackendState(ok, message) {
  backendStatus.textContent = message;
  backendStatus.classList.toggle("ok", ok);
  backendStatus.classList.toggle("error", !ok);
  actionButtons.forEach((button) => {
    button.disabled = !ok;
  });
}

async function waitForBackend() {
  let attempts = 0;
  while (attempts < 20) {
    try {
      await fetchJson("/status");
      setBackendState(true, "Online");
      addLog("Backend online.");
      return true;
    } catch {
      attempts += 1;
      setBackendState(false, "Offline");
      await new Promise((resolve) => setTimeout(resolve, 500));
    }
  }
  addLog("Backend is offline. Check Python server.");
  return false;
}

async function refreshPorts() {
  try {
    const state = await fetchJson("/ports");
    portSelect.innerHTML = "";
    const ports = state.ports || [];
    if (!ports.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "No ports detected";
      portSelect.appendChild(option);
      addLog("No serial ports detected. Check device connection and drivers.");
      connectBtn.disabled = true;
    } else {
      connectBtn.disabled = false;
      ports.forEach((port) => {
        const option = document.createElement("option");
        option.value = port;
        option.textContent = port;
        portSelect.appendChild(option);
      });
      portSelect.value = ports[0];
    }
    setStatus(state);
  } catch (error) {
    addLog(`Port scan failed: ${error.message}`);
  }
}

async function connect() {
  const port = portSelect.value;
  const baud_rate = parseInt(baudSelect.value, 10);
  if (!port) {
    addLog("Select a serial port before connecting.");
    return;
  }
  addLog(`Connecting to ${port} @ ${baud_rate}...`);
  try {
    const state = await fetchJson("/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ port, baud_rate }),
      timeoutMs: 8000,
    });
    if (state.error) {
      addLog(`Connect failed: ${state.error}`);
    } else if (state.connection_status) {
      addLog(state.connection_status);
    }
    setStatus(state);
  } catch (error) {
    addLog(`Connect failed: ${error.message}`);
  }
}

async function disconnect() {
  addLog("Disconnecting...");
  try {
    const state = await fetchJson("/disconnect", { method: "POST" });
    addLog(state.connection_status || "Disconnected");
    setStatus(state);
  } catch (error) {
    addLog(`Disconnect failed: ${error.message}`);
  }
}

async function pollResponses() {
  const state = await fetchJson("/responses");
  if (state.messages && state.messages.length) {
    state.messages.forEach(addLog);
  }
  setStatus(state);
}

async function uploadFile() {
  if (!fileInput.files.length) {
    return;
  }
  const formData = new FormData();
  formData.append("file", fileInput.files[0]);
  try {
    const state = await fetchJson("/upload", {
      method: "POST",
      body: formData,
    });
    if (state.upload_crc32) {
      addLog(`CRC32 ${state.upload_crc32}`);
    } else {
      addLog("CRC32 not available for this file.");
    }
    if (state.crc_match === "match") {
      const proceed = window.confirm(
        "Device CRC32 matches this file. Upload anyway?"
      );
      allowUpload = proceed;
      if (!proceed) {
        addLog("Upload canceled (CRC32 matches device).");
      }
    } else {
      allowUpload = true;
    }
    setStatus(state);
    startUploadBtn.disabled = !allowUpload;
  } catch (error) {
    addLog(`Upload failed: ${error.message}`);
  }
}

async function startUpload() {
  if (!allowUpload) {
    addLog("Upload blocked: CRC32 matches device.");
    return;
  }
  const state = await fetchJson("/start_upload", { method: "POST" });
  setStatus(state);
}

async function jumpApp() {
  if (jumpAppBtn.disabled) {
    return;
  }
  try {
    const state = await fetchJson("/jump_app", { method: "POST" });
    addLog("Sent JUMP_APP command.");
    setStatus(state);
  } catch (error) {
    addLog(`Jump App failed: ${error.message}`);
  }
}

refreshPortsBtn.addEventListener("click", refreshPorts);
connectBtn.addEventListener("click", () => {
  if (connectBtn.textContent === "Disconnect") {
    disconnect();
  } else {
    connect();
  }
});
uploadBtn.addEventListener("click", uploadFile);
startUploadBtn.addEventListener("click", startUpload);
jumpAppBtn.addEventListener("click", jumpApp);
clearLogBtn.addEventListener("click", () => {
  logList.innerHTML = "";
  addLog("Console cleared.");
});

setInterval(() => {
  pollResponses().catch(() => {});
}, 2000);

waitForBackend().then((ready) => {
  if (ready) {
    refreshPorts().catch(() => {});
  }
});
