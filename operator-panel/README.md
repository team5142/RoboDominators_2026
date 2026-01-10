# Operator Panel

## Prereqs
- Windows 10/11
- Node.js + npm

## Dev run
From `operator-panel/`:
- `npm install`
- `npm run dev`

## Package (Windows installer)
From `operator-panel/`:
- `npm run dist`

Output goes to `operator-panel/out/make`.

## Config
Edit `operator-panel/config.json`:
- `enableNt`: set to `true` to attempt robot connection
- `team`, `host`, `port`: connection settings

## Auto-start on Windows login (simple)
- Create a shortcut to the installed app exe under:
  `%LOCALAPPDATA%\\operator_panel\\app-<version>\\operator-panel.exe`
- Put the shortcut in:
  `shell:startup`

## Troubleshooting
- If you see the fallback white page, the operator UI file did not load.
- If NT connect errors appear, set `enableNt` to `false` to run offline.
