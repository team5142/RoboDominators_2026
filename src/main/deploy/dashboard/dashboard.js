// Dashboard State Manager - RoboDominators 5142
// Rebuilt 2026 endgame phase: single 30s END_GAME (both hubs active)
// Shift timeline: TRANSITION -> SHIFT_1 -> SHIFT_2 -> SHIFT_3 -> SHIFT_4 -> END_GAME
class Dashboard {
    constructor() {
        this.nt = null;
        this.currentPhase = 'disabled';
        this.matchTime = 150;
        this.updateInterval = null;
        this.elements = {};
        this.autoCycle = false;
        this.cycleTimer = 0;

        // Match phase state from NT
        this.hubActive = true;
        this.gamePhaseStr = 'DISABLED';
        this.gameDataReceived = false;
        this.redInactiveFirst = false;

        this.cacheElements();
        this.setupDebugMenu();
        this.initNetworkTables();
        this.startUpdateLoop();
    }

    cacheElements() {
        this.elements.alertBorder = document.getElementById('alert-border');
        this.elements.mainTimer = document.getElementById('main-timer');
        this.elements.timerMinutes = document.getElementById('timer-minutes');
        this.elements.timerSeconds = document.getElementById('timer-seconds');
        this.elements.gameSegment = document.getElementById('game-segment');
        this.elements.ledMessage = document.getElementById('led-message');

        this.elements.hubStatusBar = document.getElementById('hub-status-bar');
        this.elements.hubStatusText = document.getElementById('hub-status-text');
        this.elements.hubPhaseLabel = document.getElementById('hub-phase-label');
        this.elements.hubCountdown = document.getElementById('hub-countdown');

        this.elements.views = {
            pregame: document.getElementById('pregame-view'),
            auto: document.getElementById('auto-view'),
            teleop: document.getElementById('teleop-view'),
            endgame: document.getElementById('endgame-view')
        };

        // Pregame
        this.elements.autoSelected = document.getElementById('auto-selected');
        this.elements.poseValidation = document.getElementById('pose-validation');
        this.elements.allianceColor = document.getElementById('alliance-color');
        this.elements.dsPosition = document.getElementById('ds-position');
        this.elements.robotBattery = document.getElementById('robot-battery');
        this.elements.questTrackingStatus = document.getElementById('quest-tracking-status');
        this.elements.questBattery = document.getElementById('quest-battery');
        this.elements.questSeeded = document.getElementById('quest-seeded');
        this.elements.cameraCount = document.getElementById('camera-count');

        this.elements.subsystems = {
            drive: document.getElementById('drive-status'),
            questGyro: document.getElementById('quest-gyro-status'),
            pigeon: document.getElementById('pigeon-status'),
            tagVision: document.getElementById('tag-vision-status'),
            objectVision: document.getElementById('object-vision-status'),
            led: document.getElementById('led-status'),
            can: document.getElementById('can-status')
        };

        // Auto
        this.elements.autoRunning = document.getElementById('auto-running');
        this.elements.autoTimer = document.getElementById('auto-timer');

        // Teleop
        this.elements.driveMode = document.getElementById('drive-mode');
        this.elements.visionLock = document.getElementById('vision-lock');
        this.elements.aimError = document.getElementById('aim-error');
        this.elements.objectCount = document.getElementById('object-count');
        this.elements.objectDistance = document.getElementById('object-distance');
        this.elements.objectAngle = document.getElementById('object-angle');

        // Endgame
        this.elements.endgameStatus = document.getElementById('endgame-status');
    }

    setupDebugMenu() {
        const debugToggle = document.getElementById('debug-toggle');
        const debugMenu = document.getElementById('debug-menu');
        const autoCycleCheckbox = document.getElementById('auto-cycle');

        debugToggle.addEventListener('click', () => {
            debugMenu.classList.toggle('open');
        });

        autoCycleCheckbox.addEventListener('change', (e) => {
            this.autoCycle = e.target.checked;
            this.cycleTimer = 0;
        });
    }

    initNetworkTables() {
        // If opened from the roboRIO webserver (10.51.42.x), connect back to same host.
        // Otherwise use the fixed robot IP (works from Live Server / local dev).
        const robotHost = window.location.hostname.startsWith('10.51.42')
            ? window.location.hostname
            : '10.51.42.2';
        this.nt = new NT4Client(robotHost + ':5810', 'Dashboard_5142');

        this.nt.onConnect = () => {
            console.log('Connected to robot');
            this.subscribeToTopics();
            const el = document.getElementById('nt-status');
            if (el) { el.textContent = 'NT: CONNECTED'; el.className = 'nt-status nt-connected'; }
        };

        this.nt.onDisconnect = () => {
            console.log('Disconnected from robot');
            const el = document.getElementById('nt-status');
            if (el) { el.textContent = 'NT: DISCONNECTED'; el.className = 'nt-status nt-disconnected'; }
        };

        this.nt.onChange = (topic, value) => {
            this.handleTopicUpdate(topic, value);
        };

        this.nt.connect();
    }

    subscribeToTopics() {
        this.nt.subscribe([
            '/FMSInfo/*',
            '/SmartDashboard/*',
            '/Drive/*',
            '/Vision/*',
            '/Odometry/*',
            '/Subsystems/*',
            '/LED/*',
            '/AdvantageKit/QuestNav/*'
        ]);
    }

    handleTopicUpdate(topic, value) {
        if (topic.startsWith('/FMSInfo/')) {
            this.handleFMSUpdate(topic, value);
        } else if (topic.startsWith('/SmartDashboard/MatchPhase/')) {
            this.handleMatchPhaseUpdate(topic, value);
        } else if (topic.startsWith('/SmartDashboard/')) {
            this.handleSmartDashboardUpdate(topic, value);
        } else if (topic.startsWith('/Drive/')) {
            this.handleDriveUpdate(topic, value);
        } else if (topic.startsWith('/Vision/')) {
            this.handleVisionUpdate(topic, value);
        } else if (topic.startsWith('/Odometry/')) {
            this.handleOdometryUpdate(topic, value);
        } else if (topic.startsWith('/LED/')) {
            this.handleLEDUpdate(topic, value);
        } else if (topic.startsWith('/AdvantageKit/QuestNav/')) {
            this.handleQuestNavUpdate(topic, value);
        }
    }

    handleFMSUpdate(topic, value) {
        const key = topic.split('/').pop();
        switch (key) {
            case 'MatchTime':
                this.matchTime = value;
                break;
            case 'FMSControlData':
                // Packed control word bitmask: bit0=enabled, bit1=auto, bit2=test
                this.updateGamePhase();
                break;
        }
    }

    handleMatchPhaseUpdate(topic, value) {
        const key = topic.split('/').pop();
        switch (key) {
            case 'Phase':
                this.gamePhaseStr = value;
                this.updateShiftTimeline(value);
                break;
            case 'HubActive':
                this.hubActive = value;
                this.updateHubStatusBar();
                break;
            case 'GameDataReceived':
                this.gameDataReceived = value;
                break;
            case 'RedInactiveFirst':
                this.redInactiveFirst = value;
                this.updateShiftOwnership();
                break;
            case 'MatchTime':
                this.matchTime = value;
                break;
        }
    }

    handleSmartDashboardUpdate(topic, value) {
        // Auto chooser - SendableChooser publishes to "Auto Chooser/active"
        // Also read from Robot/AutoSelected which is published every loop
        if (topic === '/SmartDashboard/Auto Chooser/active' || topic === '/SmartDashboard/Robot/AutoSelected') {
            const name = value || 'None';
            this.elements.autoSelected.textContent = name;
            if (this.elements.autoRunning) this.elements.autoRunning.textContent = name;
            return;
        }

        // Field2d pose: [x_m, y_m, heading_rad] - use only x and y from this topic
        if (topic === '/SmartDashboard/Field/Robot') {
            if (Array.isArray(value) && value.length >= 2) {
                this.updateAllPositions('x', value[0]);
                this.updateAllPositions('y', value[1]);
            }
            return;
        }

        // Heading comes from Robot/Heading which is already normalized degrees from Java
        if (topic === '/SmartDashboard/Robot/Heading') {
            this.updateAllPositions('heading', value);
            return;
        }

        const key = topic.split('/').pop();
        switch (key) {
            case 'IsRedAlliance':
                if (topic.includes('/Robot/')) {
                    this.elements.allianceColor.textContent = value ? 'Red' : 'Blue';
                    this.elements.allianceColor.classList.remove('alliance-blue', 'alliance-red');
                    this.elements.allianceColor.classList.add(value ? 'alliance-red' : 'alliance-blue');
                    this.updateShiftOwnership();
                }
                break;
            case 'StationNumber':
                if (topic.includes('/Robot/')) {
                    this.elements.dsPosition.textContent = 'Position ' + value;
                }
                break;
            case 'Battery':
                if (topic.includes('/Robot/')) {
                    this.elements.robotBattery.textContent = value.toFixed(1) + 'V';
                    this.elements.robotBattery.style.color = value < 11.5 ? 'var(--alert-red)' : 'var(--algae)';
                } else if (topic.includes('/QuestNav/') && this.elements.questBattery) {
                    const pct = Math.round(value);
                    this.elements.questBattery.textContent = 'Battery: ' + pct + '%';
                    this.elements.questBattery.style.color = pct < 20 ? 'var(--alert-red)' : 'var(--algae)';
                }
                break;
            case 'PoseX':
            case 'PoseY':
            case 'Heading':
                // Handled by explicit topic checks above - ignore here
                break;
            case 'CameraCount':
                if (topic.includes('/Robot/') && this.elements.cameraCount) {
                    const cams = parseInt(value, 10) || 0;
                    this.elements.cameraCount.textContent = cams > 0 ? cams + ' active' : 'None';
                    this.elements.cameraCount.style.color = cams > 0 ? 'var(--algae)' : 'var(--alert-red)';
                }
                break;
        }
    }

    handleDriveUpdate(topic, value) {
        const key = topic.split('/').pop();
        if (key === 'DriveMode') {
            this.elements.driveMode.textContent = value.toUpperCase();
            const egMode = document.getElementById('endgame-drive-mode');
            if (egMode) egMode.textContent = value.toUpperCase();
        } else if (key === 'Status') {
            this.updateSubsystemStatus('drive', value);
        }
    }

    handleVisionUpdate(topic, value) {
        const key = topic.split('/').pop();
        switch (key) {
            case 'TagVision_Status':
                this.updateSubsystemStatus('tagVision', value);
                break;
            case 'ObjectVision_Status':
                this.updateSubsystemStatus('objectVision', value);
                break;
            case 'VisionLock': {
                const lockText = value ? 'LOCKED' : 'NONE';
                this.elements.visionLock.textContent = lockText;
                const egLock = document.getElementById('endgame-vision-lock');
                if (egLock) egLock.textContent = lockText;
                break;
            }
            case 'AimError': {
                const errText = 'Error: ' + value.toFixed(1) + 'deg';
                this.elements.aimError.textContent = errText;
                const egAim = document.getElementById('endgame-aim-error');
                if (egAim) egAim.textContent = errText;
                break;
            }
            case 'VisibleTags':
                this.updateAllTagCounts(value);
                break;
            case 'ObjectCount': {
                const cnt = value.toString();
                this.elements.objectCount.textContent = cnt;
                const egCnt = document.getElementById('endgame-object-count');
                if (egCnt) egCnt.textContent = cnt;
                break;
            }
            case 'ObjectDistance': {
                const distText = value ? value.toFixed(2) + 'm' : '--';
                this.elements.objectDistance.textContent = distText;
                const egDist = document.getElementById('endgame-object-distance');
                if (egDist) egDist.textContent = distText;
                break;
            }
            case 'ObjectAngle': {
                const angText = value ? value.toFixed(1) + 'deg' : '--';
                this.elements.objectAngle.textContent = angText;
                const egAng = document.getElementById('endgame-object-angle');
                if (egAng) egAng.textContent = angText;
                break;
            }
            case 'CameraCount':
                this.elements.cameraCount.textContent = value + '/4';
                break;
        }
    }

    handleOdometryUpdate(topic, value) {
        const key = topic.split('/').pop();
        switch (key) {
            case 'AvgError': this.updateAllPoseErrors(value); break;
            case 'PoseQuality': {
                const quality = value > 0.8 ? 'GOOD' : value > 0.5 ? 'FAIR' : 'POOR';
                ['teleop-pose-confidence', 'endgame-pose-confidence'].forEach(id => {
                    const el = document.getElementById(id);
                    if (el) el.textContent = quality;
                });
                break;
            }
        }
    }

    handleLEDUpdate(topic, value) {
        const key = topic.split('/').pop();
        if (key === 'Message') {
            this.elements.ledMessage.textContent = value || 'READY';
        } else if (key === 'Status') {
            this.updateSubsystemStatus('led', value);
        }
    }

    handleQuestNavUpdate(topic, value) {
        const key = topic.split('/').pop();
        switch (key) {
            case 'Connected':
                this.updateSubsystemStatus('questGyro', value ? 'OK' : 'ERROR');
                break;
            case 'Tracking':
                if (this.elements.questTrackingStatus) {
                    this.elements.questTrackingStatus.textContent = value ? 'TRACKING' : 'NO TRACK';
                    this.elements.questTrackingStatus.style.color = value ? 'var(--algae)' : 'var(--alert-red)';
                }
                break;
            case 'Seeded':
                if (this.elements.questSeeded) {
                    this.elements.questSeeded.textContent = 'Seeded: ' + (value ? 'YES' : 'NO');
                    this.elements.questSeeded.style.color = value ? 'var(--algae)' : 'var(--alert-yellow)';
                }
                break;
        }
    }

    // Hub status bar - shows HUB ACTIVE / HUB INACTIVE during teleop/endgame
    updateHubStatusBar() {
        if (!this.elements.hubStatusBar) return;
        if (this.currentPhase === 'teleop' || this.currentPhase === 'endgame') {
            this.elements.hubStatusBar.style.display = 'flex';
        }
        if (this.hubActive) {
            this.elements.hubStatusBar.className = 'hub-status-bar hub-active';
            this.elements.hubStatusText.textContent = 'HUB ACTIVE - SHOOT';
        } else {
            this.elements.hubStatusBar.className = 'hub-status-bar hub-inactive';
            this.elements.hubStatusText.textContent = 'HUB INACTIVE - HOLD';
        }
    }

    // Highlight the current shift block and dim the rest
    updateShiftTimeline(phaseStr) {
        const blocks = document.querySelectorAll('.shift-block');
        blocks.forEach(b => {
            b.classList.remove('shift-current', 'shift-done');
            const blockPhase = b.getAttribute('data-phase');
            const isDone = this.isPhaseBeforeCurrent(blockPhase, phaseStr);
            if (isDone) b.classList.add('shift-done');
            if (blockPhase === phaseStr) b.classList.add('shift-current');
        });
        if (this.elements.hubPhaseLabel) {
            this.elements.hubPhaseLabel.textContent = this.formatPhaseName(phaseStr);
        }
    }

    // Color shift blocks green (we own) or red (opponent owns) based on game data
    updateShiftOwnership() {
        // redInactiveFirst: Red is inactive in shifts 1+3, active in 2+4
        // Blue is the opposite
        const phases = {
            'SHIFT_1': !this.redInactiveFirst, // true = blue active
            'SHIFT_2': this.redInactiveFirst,
            'SHIFT_3': !this.redInactiveFirst,
            'SHIFT_4': this.redInactiveFirst
        };

        const isRed = this.elements.allianceColor &&
            this.elements.allianceColor.classList.contains('alliance-red');

        Object.entries(phases).forEach(([phase, blueActive]) => {
            const block = document.getElementById('shift-block-' + phase.toLowerCase().replace('_', '-'));
            if (!block) return;
            const weOwn = isRed ? !blueActive : blueActive;
            block.classList.remove('shift-we-own', 'shift-opponent-owns');
            block.classList.add(weOwn ? 'shift-we-own' : 'shift-opponent-owns');
        });
    }

    isPhaseBeforeCurrent(blockPhase, currentPhaseStr) {
        const order = ['TRANSITION_SHIFT', 'SHIFT_1', 'SHIFT_2', 'SHIFT_3', 'SHIFT_4', 'END_GAME'];
        const blockIdx = order.indexOf(blockPhase);
        const currentIdx = order.indexOf(currentPhaseStr);
        return blockIdx !== -1 && currentIdx !== -1 && blockIdx < currentIdx;
    }

    formatPhaseName(phaseStr) {
        const names = {
            'TRANSITION_SHIFT': 'TRANSITION',
            'SHIFT_1': 'SHIFT 1',
            'SHIFT_2': 'SHIFT 2',
            'SHIFT_3': 'SHIFT 3',
            'SHIFT_4': 'SHIFT 4',
            'END_GAME': 'END GAME',
            'AUTO': 'AUTO',
            'DISABLED': 'DISABLED'
        };
        return names[phaseStr] || phaseStr;
    }

    updateSubsystemStatus(subsystem, status) {
        const element = this.elements.subsystems[subsystem];
        if (element) {
            element.className = 'status-indicator';
            if (status === 'OK' || status === true) {
                element.classList.add('ok');
            } else if (status === 'WARN') {
                element.classList.add('warn');
            } else {
                element.classList.add('error');
            }
        }
    }

    updateAllTagCounts(count) {
        ['visible-tags', 'auto-visible-tags', 'teleop-visible-tags', 'endgame-visible-tags'].forEach(id => {
            const el = document.getElementById(id);
            if (el) el.textContent = 'Tags: ' + count;
        });
    }

    updateAllPositions(coord, value) {
        const prefixes = ['', 'auto-', 'teleop-', 'endgame-'];
        prefixes.forEach(prefix => {
            if (coord === 'x') {
                const el = document.getElementById(prefix + 'field-x');
                if (el) el.textContent = 'X: ' + value.toFixed(2) + 'm';
            } else if (coord === 'y') {
                const el = document.getElementById(prefix + 'field-y');
                if (el) el.textContent = 'Y: ' + value.toFixed(2) + 'm';
            } else if (coord === 'heading') {
                const el = document.getElementById(prefix + 'field-heading');
                if (el) el.textContent = 'θ: ' + value.toFixed(1) + 'deg';
            }
        });
    }

    updateAllPoseErrors(error) {
        ['pose-error', 'auto-pose-error', 'teleop-pose-error'].forEach(id => {
            const el = document.getElementById(id);
            if (el) el.textContent = error.toFixed(2) + 'm';
        });
    }

    setPhase(phase) {
        this.switchPhase(phase);
        switch (phase) {
            case 'disabled': this.matchTime = 150; break;
            case 'auto':     this.matchTime = 150; break;
            case 'teleop':   this.matchTime = 135; break;
            case 'endgame':  this.matchTime = 30;  break;
        }
    }

    startUpdateLoop() {
        this.updateInterval = setInterval(() => {
            this.updateGamePhase();
            this.updateTimer();

            if (this.autoCycle) {
                this.cycleTimer++;
                if (this.cycleTimer > 100) {
                    this.cycleTimer = 0;
                    const phases = ['disabled', 'auto', 'teleop', 'endgame'];
                    const next = (phases.indexOf(this.currentPhase) + 1) % phases.length;
                    this.setPhase(phases[next]);
                }
            }
        }, 50);
    }

    updateGamePhase() {
        // Auto-cycle overrides NT when enabled (debug tool only)
        if (this.autoCycle) return;

        if (this.nt && this.nt.connected) {
            // FMSControlData is a packed bitmask: bit0=enabled, bit1=autonomous, bit2=test
            const controlData = this.nt.getValue('/FMSInfo/FMSControlData', 0);
            const isEnabled  = (controlData & 0x01) !== 0;
            const isAuto     = (controlData & 0x02) !== 0;
            const isTest     = (controlData & 0x04) !== 0;
            const isTeleop   = isEnabled && !isAuto && !isTest;

            let newPhase = 'disabled';
            if (!isEnabled) {
                newPhase = 'disabled';
            } else if (isAuto) {
                newPhase = 'auto';
            } else if (isTeleop) {
                // Single endgame phase at 30s remaining
                newPhase = this.matchTime <= 30 ? 'endgame' : 'teleop';
            }

            if (newPhase !== this.currentPhase) this.switchPhase(newPhase);
        }
        // Not connected: stay on whatever phase was last set (disabled by default)
    }

    switchPhase(newPhase) {
        const logo = document.querySelector('.logo');

        if (newPhase !== this.currentPhase) {
            this.elements.alertBorder.className = 'alert-border ' + newPhase;
            logo.className = 'logo ' + newPhase;
            this.elements.alertBorder.classList.add('blink');
            logo.classList.add('blink');
            setTimeout(() => {
                this.elements.alertBorder.classList.remove('blink');
                logo.classList.remove('blink');
            }, 1000);
        }

        this.currentPhase = newPhase;

        const phaseLabels = {
            disabled: 'DISABLED',
            auto: 'AUTONOMOUS',
            teleop: 'TELEOP',
            endgame: 'END GAME'
        };
        this.elements.gameSegment.textContent = phaseLabels[newPhase] || newPhase.toUpperCase();
        this.elements.alertBorder.className = 'alert-border ' + newPhase;

        // Hub status bar - show only during teleop and endgame
        if (this.elements.hubStatusBar) {
            const showHub = newPhase === 'teleop' || newPhase === 'endgame';
            this.elements.hubStatusBar.style.display = showHub ? 'flex' : 'none';
            if (showHub) this.updateHubStatusBar();
        }

        Object.values(this.elements.views).forEach(v => v.classList.remove('active', 'faded'));

        switch (newPhase) {
            case 'disabled':
                this.elements.views.pregame.classList.add('active');
                this.elements.mainTimer.classList.remove('endgame-size');
                break;
            case 'auto':
                this.elements.views.auto.classList.add('active');
                this.elements.mainTimer.classList.remove('endgame-size');
                break;
            case 'teleop':
                this.elements.views.teleop.classList.add('active');
                this.elements.mainTimer.classList.remove('endgame-size');
                break;
            case 'endgame':
                this.elements.views.endgame.classList.add('active');
                this.elements.mainTimer.classList.add('endgame-size');
                break;
        }
    }

    updateTimer() {
        const minutes = Math.floor(this.matchTime / 60);
        const seconds = Math.floor(this.matchTime % 60);
        this.elements.timerMinutes.textContent = minutes.toString().padStart(2, '0');
        this.elements.timerSeconds.textContent = seconds.toString().padStart(2, '0');

        if (this.currentPhase === 'auto') {
            const autoTime = Math.max(0, Math.floor(this.matchTime - 135));
            this.elements.autoTimer.textContent = autoTime.toString();
        }

        // Update hub countdown text
        if (this.elements.hubCountdown && (this.currentPhase === 'teleop' || this.currentPhase === 'endgame')) {
            const nextEdge = this.getNextPhaseEdge();
            this.elements.hubCountdown.textContent = nextEdge !== null ? nextEdge.toFixed(0) + 's' : '--s';
        }

        // Count down locally when FMS is not attached (practice / pit use)
        if (this.currentPhase !== 'disabled') {
            const fmsAttached = (this.nt.getValue('/FMSInfo/FMSControlData', 0) & 0x10) !== 0;
            if (!fmsAttached) {
                this.matchTime = Math.max(0, this.matchTime - 0.05);
            }
        }
    }

    // Returns seconds until the next shift boundary (for hub countdown display)
    getNextPhaseEdge() {
        const boundaries = [120, 95, 70, 45, 30, 0];
        for (const b of boundaries) {
            if (this.matchTime > b) return this.matchTime - b;
        }
        return null;
    }
}

let dashboard;
window.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing RoboDominators 5142 Dashboard');
    dashboard = new Dashboard();
});
