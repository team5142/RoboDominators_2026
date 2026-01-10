const path = require('path');
const fs = require('fs');
const { app, BrowserWindow, ipcMain } = require('electron');

const { NtBridge } = require('./nt/NtBridge');

process.on('unhandledRejection', (reason) => {
  console.error('Unhandled promise rejection:', reason);
});

function loadConfig() {
  const cfgPath = path.join(__dirname, '..', 'config.json');
  try {
    const txt = fs.readFileSync(cfgPath, 'utf8');
    const cfg = JSON.parse(txt);
    cfg._cfgPath = cfgPath;
    return cfg;
  } catch {
    return { team: 5142, host: '10.51.42.2', port: 5810, enableNt: false, _cfgPath: cfgPath };
  }
}

function teamToHost(team) {
  const t = Number(team);
  if (!Number.isFinite(t) || t < 1 || t > 9999) return null;
  const a = Math.floor(t / 100);
  const b = t % 100;
  return `10.${a}.${b}.2`;
}

let mainWindow = null;
let nt = null;
let _ntEnabled = false;

function resolveRepoFile(relFromRepoRoot) {
  // In dev, cwd is operator-panel. In packaged, this will need adjustment.
  const repoRoot = path.resolve(process.cwd(), '..');
  return path.resolve(repoRoot, relFromRepoRoot);
}

function fileExists(p) {
  try {
    return fs.existsSync(p);
  } catch {
    return false;
  }
}

function firstExistingPath(paths) {
  for (let i = 0; i < paths.length; i += 1) {
    const p = paths[i];
    if (fileExists(p)) return p;
  }
  return null;
}

function getOperatorUiPath() {
  const packaged = firstExistingPath([
    path.join(process.resourcesPath, 'assets', 'operator', 'operator.html'),
    path.join(process.resourcesPath, 'operator', 'operator.html'),
    path.join(process.resourcesPath, 'assets', 'assets', 'operator', 'operator.html'),
  ]);
  if (packaged) return packaged;

  return resolveRepoFile('src/main/deploy/operator/operator.html');
}

async function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1024,
    height: 600,
    useContentSize: true,
    resizable: false,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      preload: MAIN_WINDOW_PRELOAD_WEBPACK_ENTRY,
    },
  });

  const uiPath = getOperatorUiPath();

  try {
    await mainWindow.loadFile(uiPath);
  } catch (e) {
    await mainWindow.loadURL(MAIN_WINDOW_WEBPACK_ENTRY);
    console.error('Failed to load operator UI file:', uiPath, e);
  }
}

function setupNt() {
  const cfg = loadConfig();

  const teamCfg = Number(cfg.team || 0);
  const team = Number.isFinite(teamCfg) && teamCfg >= 1 && teamCfg <= 9999 ? teamCfg : 5142;

  const hostCfg = cfg.host ? String(cfg.host) : '';
  const host = hostCfg || teamToHost(team) || '10.51.42.2';

  const port = Number(cfg.port || 5810);

  nt = new NtBridge({ team, host, port });

  nt.onConnection((connected) => {
    if (!mainWindow) return;
    mainWindow.webContents.send('opPanel:connection', { connected });
  });

  nt.onUiModelJson((json) => {
    if (!mainWindow) return;
    mainWindow.webContents.send('opPanel:uiModel', { json });
  });

  nt.onAckSeq((seq) => {
    if (!mainWindow) return;
    mainWindow.webContents.send('opPanel:ack', { seq });
  });

  try {
    nt.connect();
  } catch (e) {
    console.error('NT connect failed:', e);
  }
}

let _seq = 0;

function nextSeq() {
  _seq += 1;
  return _seq;
}

function logDev(...args) {
  if (app.isPackaged) return;
  console.log(...args);
}

function setupIpc() {
  ipcMain.handle('opPanel:sendRequest', async (_evt, payload) => {
    const name = payload && payload.name ? String(payload.name) : '';
    const args = payload && payload.args != null ? payload.args : null;

    logDev('UI request:', name, args);

    if (!_ntEnabled) {
      return { ok: true, offline: true };
    }

    if (!nt) return { ok: false, error: 'NT not ready' };
    if (!name) return { ok: false, error: 'Missing name' };

    const seq = nextSeq();
    let argsJson = '';

    if (payload && payload.args != null) {
      try {
        argsJson = JSON.stringify(payload.args);
      } catch (e) {
        return { ok: false, error: String(e && e.message ? e.message : e) };
      }
    }

    try {
      nt.sendRequest(name, seq, argsJson);
      return { ok: true, seq };
    } catch (e) {
      return { ok: false, error: String(e && e.message ? e.message : e) };
    }
  });
}

app.whenReady().then(async () => {
  await createWindow();
  setupIpc();

  const cfg = loadConfig();
  _ntEnabled = cfg && cfg.enableNt === true;

  logDev('Config path:', cfg && cfg._cfgPath ? cfg._cfgPath : '(unknown)');
  logDev('enableNt:', _ntEnabled);

  if (_ntEnabled) {
    setupNt();
  } else if (mainWindow) {
    mainWindow.webContents.send('opPanel:connection', { connected: false });
  }
});

app.on('window-all-closed', () => {
  app.quit();
});