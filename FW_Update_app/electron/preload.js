const { contextBridge } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  backendUrl: "http://127.0.0.1:8000",
});
