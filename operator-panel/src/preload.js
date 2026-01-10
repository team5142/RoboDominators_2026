// See the Electron documentation for details on how to use preload scripts:
// https://www.electronjs.org/docs/latest/tutorial/process-model#preload-scripts

const { contextBridge, ipcRenderer } = require('electron');

function on(channel, cb) {
  const handler = (_evt, payload) => cb(payload);
  ipcRenderer.on(channel, handler);
  return () => ipcRenderer.removeListener(channel, handler);
}

contextBridge.exposeInMainWorld('opPanel', {
  sendRequest: (name, args) => ipcRenderer.invoke('opPanel:sendRequest', { name, args }),
  onConnection: (cb) => on('opPanel:connection', cb),
  onUiModelUpdate: (cb) => on('opPanel:uiModel', cb),
  onAck: (cb) => on('opPanel:ack', cb),
});
