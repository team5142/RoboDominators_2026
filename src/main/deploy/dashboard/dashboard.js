// Dashboard State Manager
class Dashboard {
    constructor() {
        this.nt = null;
        this.currentPhase = 'disabled';
        this.matchTime = 150; // 2:30 in seconds
        this.isCompetition = true; // Set to false for testing
        this.updateInterval = null;
        this.elements = {};
        this.demoMode = true; // Start in demo mode
        this.autoCycle = false;
        this.cycleTimer = 0;
        
        // Cache DOM elements
        this.cacheElements();
        
        // Set up debug menu
        this.setupDebugMenu();
        
        // Initialize NetworkTables
        this.initNetworkTables();
        
        // Initialize demo data
        this.initDemoData();
        
        // Start update loop
        this.startUpdateLoop();
    }

    cacheElements() {
        // Global elements
        this.elements.alertBorder = document.getElementById('alert-border');
        this.elements.mainTimer = document.getElementById('main-timer');
        this.elements.timerMinutes = document.getElementById('timer-minutes');
        this.elements.timerSeconds = document.getElementById('timer-seconds');
        this.elements.gameSegment = document.getElementById('game-segment');
        this.elements.ledMessage = document.getElementById('led-message');
        
        // View containers
        this.elements.views = {
            pregame: document.getElementById('pregame-view'),
            auto: document.getElementById('auto-view'),
            teleop: document.getElementById('teleop-view'),
            endgame1: document.getElementById('endgame1-view'),
            endgame2: document.getElementById('endgame2-view')
        };
        
        // Pregame elements
        this.elements.autoSelected = document.getElementById('auto-selected');
        this.elements.allianceColor = document.getElementById('alliance-color');
        this.elements.dsPosition = document.getElementById('ds-position');
        this.elements.robotBattery = document.getElementById('robot-battery');
        this.elements.questBattery = document.getElementById('quest-battery');
        this.elements.cameraCount = document.getElementById('camera-count');
        
        // Subsystem status indicators
        this.elements.subsystems = {
            drive: document.getElementById('drive-status'),
            questGyro: document.getElementById('quest-gyro-status'),
            pigeon: document.getElementById('pigeon-status'),
            tagVision: document.getElementById('tag-vision-status'),
            objectVision: document.getElementById('object-vision-status'),
            led: document.getElementById('led-status'),
            can: document.getElementById('can-status')
        };
        
        // Auto view elements
        this.elements.autoRunning = document.getElementById('auto-running');
        this.elements.autoTimer = document.getElementById('auto-timer');
        
        // Teleop elements
        this.elements.driveMode = document.getElementById('drive-mode');
        this.elements.visionLock = document.getElementById('vision-lock');
        this.elements.aimError = document.getElementById('aim-error');
        this.elements.objectCount = document.getElementById('object-count');
        this.elements.objectDistance = document.getElementById('object-distance');
        this.elements.objectAngle = document.getElementById('object-angle');
        
        // Endgame elements
        this.elements.endgameStatus = document.getElementById('endgame-status');
        this.elements.endgame2Status = document.getElementById('endgame2-status');
    }

    setupDebugMenu() {
        const debugToggle = document.getElementById('debug-toggle');
        const debugMenu = document.getElementById('debug-menu');
        const demoModeCheckbox = document.getElementById('demo-mode');
        const autoCycleCheckbox = document.getElementById('auto-cycle');
        
        debugToggle.addEventListener('click', () => {
            debugMenu.classList.toggle('open');
        });
        
        demoModeCheckbox.addEventListener('change', (e) => {
            this.demoMode = e.target.checked;
            if (this.demoMode) {
                this.initDemoData();
            }
        });
        
        autoCycleCheckbox.addEventListener('change', (e) => {
            this.autoCycle = e.target.checked;
            this.cycleTimer = 0;
        });
    }

    initNetworkTables() {
        // Try to connect to robot at 10.51.42.2 (Team 5142)
        this.nt = new NT4Client('10.51.42.2:5810', 'Dashboard_5142');
        
        // Set up connection callbacks
        this.nt.onConnect = () => {
            console.log('Connected to robot');
            this.subscribeToTopics();
        };
        
        this.nt.onDisconnect = () => {
            console.log('Disconnected from robot');
        };
        
        this.nt.onChange = (topic, value) => {
            this.handleTopicUpdate(topic, value);
        };
        
        // Start connection
        this.nt.connect();
    }

    subscribeToTopics() {
        // Subscribe to all relevant topics
        this.nt.subscribe([
            '/FMSInfo/*',
            '/SmartDashboard/*',
            '/Drive/*',
            '/Vision/*',
            '/Odometry/*',
            '/Subsystems/*',
            '/LED/*'
        ]);
    }

    handleTopicUpdate(topic, value) {
        // Handle FMS info
        if (topic.startsWith('/FMSInfo/')) {
            this.handleFMSUpdate(topic, value);
        }
        // Handle robot state
        else if (topic.startsWith('/SmartDashboard/')) {
            this.handleSmartDashboardUpdate(topic, value);
        }
        // Handle drive updates
        else if (topic.startsWith('/Drive/')) {
            this.handleDriveUpdate(topic, value);
        }
        // Handle vision updates
        else if (topic.startsWith('/Vision/')) {
            this.handleVisionUpdate(topic, value);
        }
        // Handle odometry updates
        else if (topic.startsWith('/Odometry/')) {
            this.handleOdometryUpdate(topic, value);
        }
        // Handle LED updates
        else if (topic.startsWith('/LED/')) {
            this.handleLEDUpdate(topic, value);
        }
    }

    handleFMSUpdate(topic, value) {
        const key = topic.split('/').pop();
        
        switch(key) {
            case 'IsRedAlliance':
                const allianceText = value ? 'Red' : 'Blue';
                this.elements.allianceColor.textContent = allianceText;
                // Add color class
                this.elements.allianceColor.classList.remove('alliance-blue', 'alliance-red');
                this.elements.allianceColor.classList.add(value ? 'alliance-red' : 'alliance-blue');
                break;
            case 'StationNumber':
                this.elements.dsPosition.textContent = `Position ${value}`;
                break;
            case 'MatchTime':
                this.matchTime = value;
                break;
        }
    }

    handleSmartDashboardUpdate(topic, value) {
        const key = topic.split('/').pop();
        
        switch(key) {
            case 'Auto_Selected':
                this.elements.autoSelected.textContent = value || 'None';
                if (this.elements.autoRunning) {
                    this.elements.autoRunning.textContent = value || 'None';
                }
                break;
            case 'Robot_Battery':
                const voltage = value.toFixed(1);
                this.elements.robotBattery.textContent = `${voltage}V`;
                this.elements.robotBattery.style.color = value < 11.5 ? 'var(--alert-red)' : 'var(--algae)';
                break;
            case 'Quest_Battery':
                this.elements.questBattery.textContent = value ? `${value}%` : '--';
                break;
        }
    }

    handleDriveUpdate(topic, value) {
        const key = topic.split('/').pop();
        
        switch(key) {
            case 'DriveMode':
                this.elements.driveMode.textContent = value.toUpperCase();
                break;
            case 'Status':
                this.updateSubsystemStatus('drive', value);
                break;
        }
    }

    handleVisionUpdate(topic, value) {
        const key = topic.split('/').pop();
        
        switch(key) {
            case 'TagVision_Status':
                this.updateSubsystemStatus('tagVision', value);
                break;
            case 'ObjectVision_Status':
                this.updateSubsystemStatus('objectVision', value);
                break;
            case 'VisionLock':
                const lockText = value ? 'LOCKED' : 'NONE';
                this.elements.visionLock.textContent = lockText;
                // Sync to endgame views
                const eg1Lock = document.getElementById('endgame1-vision-lock');
                if (eg1Lock) eg1Lock.textContent = lockText;
                const eg2Lock = document.getElementById('endgame2-vision-lock');
                if (eg2Lock) eg2Lock.textContent = lockText;
                break;
            case 'AimError':
                const errorText = `Error: ${value.toFixed(1)}°`;
                this.elements.aimError.textContent = errorText;
                const eg1Aim = document.getElementById('endgame1-aim-error');
                if (eg1Aim) eg1Aim.textContent = errorText;
                break;
            case 'VisibleTags':
                this.updateAllTagCounts(value);
                break;
            case 'ObjectCount':
                this.elements.objectCount.textContent = value;
                const eg1Count = document.getElementById('endgame1-object-count');
                if (eg1Count) eg1Count.textContent = value;
                const eg2Count = document.getElementById('endgame2-object-count');
                if (eg2Count) eg2Count.textContent = value;
                break;
            case 'ObjectDistance':
                const distText = value ? `${value.toFixed(2)}m` : '--';
                this.elements.objectDistance.textContent = distText;
                const eg1Dist = document.getElementById('endgame1-object-distance');
                if (eg1Dist) eg1Dist.textContent = distText;
                const eg2Dist = document.getElementById('endgame2-object-distance');
                if (eg2Dist) eg2Dist.textContent = distText;
                break;
            case 'ObjectAngle':
                const angleText = value ? `${value.toFixed(1)}°` : '--';
                this.elements.objectAngle.textContent = angleText;
                const eg1Angle = document.getElementById('endgame1-object-angle');
                if (eg1Angle) eg1Angle.textContent = angleText;
                const eg2Angle = document.getElementById('endgame2-object-angle');
                if (eg2Angle) eg2Angle.textContent = angleText;
                break;
            case 'CameraCount':
                this.elements.cameraCount.textContent = `${value}/4`;
                break;
        }
    }

    handleOdometryUpdate(topic, value) {
        const key = topic.split('/').pop();
        
        switch(key) {
            case 'X':
                this.updateAllPositions('x', value);
                break;
            case 'Y':
                this.updateAllPositions('y', value);
                break;
            case 'Heading':
                this.updateAllPositions('heading', value);
                break;
            case 'AvgError':
                this.updateAllPoseErrors(value);
                break;
            case 'PoseQuality':
                const quality = value > 0.8 ? 'GOOD' : value > 0.5 ? 'FAIR' : 'POOR';
                if (document.getElementById('teleop-pose-confidence')) {
                    document.getElementById('teleop-pose-confidence').textContent = quality;
                }
                if (document.getElementById('endgame1-pose-confidence')) {
                    document.getElementById('endgame1-pose-confidence').textContent = quality;
                }
                if (document.getElementById('endgame2-pose-confidence')) {
                    document.getElementById('endgame2-pose-confidence').textContent = quality;
                }
                break;
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
        const tagElements = [
            'visible-tags',
            'auto-visible-tags',
            'teleop-visible-tags',
            'endgame1-visible-tags'
        ];
        
        tagElements.forEach(id => {
            const elem = document.getElementById(id);
            if (elem) elem.textContent = `Tags: ${count}`;
        });
    }

    updateAllPositions(coord, value) {
        const prefixes = ['', 'auto-', 'teleop-', 'endgame1-', 'endgame2-'];
        
        prefixes.forEach(prefix => {
            if (coord === 'x') {
                const elem = document.getElementById(`${prefix}field-x`);
                if (elem) elem.textContent = `X: ${value.toFixed(2)}m`;
            } else if (coord === 'y') {
                const elem = document.getElementById(`${prefix}field-y`);
                if (elem) elem.textContent = `Y: ${value.toFixed(2)}m`;
            } else if (coord === 'heading') {
                const elem = document.getElementById(`${prefix}field-heading`);
                if (elem) elem.textContent = `θ: ${value.toFixed(1)}°`;
            }
        });
    }

    updateAllPoseErrors(error) {
        const errorElements = [
            'pose-error',
            'auto-pose-error',
            'teleop-pose-error'
        ];
        
        errorElements.forEach(id => {
            const elem = document.getElementById(id);
            if (elem) elem.textContent = `${error.toFixed(2)}m`;
        });
    }

    initDemoData() {
        // Set initial demo values
        this.elements.autoSelected.textContent = 'Center 4-Note';
        this.elements.allianceColor.textContent = 'Blue';
        this.elements.allianceColor.classList.add('alliance-blue'); // Add color class for demo
        this.elements.dsPosition.textContent = 'Position 2';
        this.elements.robotBattery.textContent = '12.4V';
        this.elements.robotBattery.style.color = 'var(--algae)';
        this.elements.questBattery.textContent = '87%';
        this.elements.cameraCount.textContent = '4/4';
        
        // Set subsystem statuses
        this.updateSubsystemStatus('drive', 'OK');
        this.updateSubsystemStatus('questGyro', 'OK');
        this.updateSubsystemStatus('pigeon', 'OK');
        this.updateSubsystemStatus('tagVision', 'OK');
        this.updateSubsystemStatus('objectVision', 'OK');
        this.updateSubsystemStatus('led', 'OK');
        this.updateSubsystemStatus('can', 'OK');
        
        // Set position data
        this.updateAllPositions('x', 1.54);
        this.updateAllPositions('y', 5.55);
        this.updateAllPositions('heading', 45.3);
        
        // Set odometry data
        this.updateAllPoseErrors(0.03);
        this.updateAllTagCounts(3);
        
        // Set drive mode
        this.elements.driveMode.textContent = 'NORMAL';
        
        // Set vision data
        this.elements.visionLock.textContent = 'NONE';
        this.elements.aimError.textContent = 'Error: --';
        
        // Set object detection data
        this.elements.objectCount.textContent = '2';
        this.elements.objectDistance.textContent = '1.45m';
        this.elements.objectAngle.textContent = '12.5°';
        
        // Set pose quality
        const poseConfidence = document.getElementById('teleop-pose-confidence');
        if (poseConfidence) poseConfidence.textContent = 'GOOD';
        const endgamePoseConfidence = document.getElementById('endgame1-pose-confidence');
        if (endgamePoseConfidence) endgamePoseConfidence.textContent = 'GOOD';
        
        // Set LED message
        this.elements.ledMessage.textContent = 'READY TO ROCK';
        
        // Set auto running
        this.elements.autoRunning.textContent = 'Center 4-Note';
    }

    updateDemoData() {
        if (!this.demoMode) return;
        
        // Simulate some changing values
        const time = Date.now() / 1000;
        
        // Oscillating heading
        const heading = 45 + Math.sin(time * 0.5) * 15;
        this.updateAllPositions('heading', heading);
        
        // Slowly changing position
        const x = 1.54 + Math.sin(time * 0.2) * 0.5;
        const y = 5.55 + Math.cos(time * 0.2) * 0.3;
        this.updateAllPositions('x', x);
        this.updateAllPositions('y', y);
        
        // Varying object distance
        if (this.currentPhase === 'teleop' || this.currentPhase === 'endgame1' || this.currentPhase === 'endgame2') {
            const distance = 1.2 + Math.abs(Math.sin(time * 0.8)) * 0.8;
            const distText = `${distance.toFixed(2)}m`;
            this.elements.objectDistance.textContent = distText;
            const eg1Dist = document.getElementById('endgame1-object-distance');
            if (eg1Dist) eg1Dist.textContent = distText;
            const eg2Dist = document.getElementById('endgame2-object-distance');
            if (eg2Dist) eg2Dist.textContent = distText;
            
            const angle = Math.sin(time * 0.6) * 25;
            const angleText = `${angle.toFixed(1)}°`;
            this.elements.objectAngle.textContent = angleText;
            const eg1Angle = document.getElementById('endgame1-object-angle');
            if (eg1Angle) eg1Angle.textContent = angleText;
            const eg2Angle = document.getElementById('endgame2-object-angle');
            if (eg2Angle) eg2Angle.textContent = angleText;
        }
        
        // Simulate battery drain
        const batteryVoltage = 12.6 - (150 - this.matchTime) * 0.008;
        this.elements.robotBattery.textContent = `${batteryVoltage.toFixed(1)}V`;
        this.elements.robotBattery.style.color = batteryVoltage < 11.5 ? 'var(--alert-red)' : 'var(--algae)';
        
        // Simulate Quest battery drain
        const questBattery = Math.max(0, 100 - (150 - this.matchTime) * 0.4);
        this.elements.questBattery.textContent = `${Math.floor(questBattery)}%`;
        
        // Sync drive mode to endgame
        const eg1DriveMode = document.getElementById('endgame1-drive-mode');
        if (eg1DriveMode) eg1DriveMode.textContent = this.elements.driveMode.textContent;
        const eg2DriveMode = document.getElementById('endgame2-drive-mode');
        if (eg2DriveMode) eg2DriveMode.textContent = this.elements.driveMode.textContent;
        
        // Randomly change vision lock in teleop
        if ((this.currentPhase === 'teleop' || this.currentPhase === 'endgame1' || this.currentPhase === 'endgame2') && Math.random() > 0.98) {
            const locked = this.elements.visionLock.textContent === 'LOCKED';
            const lockText = locked ? 'NONE' : 'LOCKED';
            const errorText = locked ? 'Error: --' : `Error: ${(Math.random() * 5).toFixed(1)}°`;
            
            this.elements.visionLock.textContent = lockText;
            this.elements.aimError.textContent = errorText;
            
            const eg1Lock = document.getElementById('endgame1-vision-lock');
            if (eg1Lock) eg1Lock.textContent = lockText;
            const eg2Lock = document.getElementById('endgame2-vision-lock');
            if (eg2Lock) eg2Lock.textContent = lockText;
            const eg1Aim = document.getElementById('endgame1-aim-error');
            if (eg1Aim) eg1Aim.textContent = errorText;
        }
    }

    setPhase(phase) {
        // Manual phase override for debugging
        this.switchPhase(phase);
        
        // Reset timers based on phase - use actual match timing
        switch(phase) {
            case 'disabled':
                this.matchTime = 150; // 2:30 (doesn't count down)
                break;
            case 'auto':
                this.matchTime = 150; // Starts at 2:30, counts to 2:15
                break;
            case 'teleop':
                this.matchTime = 135; // Starts at 2:15, counts to 0:31
                break;
            case 'endgame1':
                this.matchTime = 30; // Starts at 0:30, counts to 0:16
                break;
            case 'endgame2':
                this.matchTime = 15; // Starts at 0:15, counts to 0:00
                break;
        }
    }

    startUpdateLoop() {
        this.updateInterval = setInterval(() => {
            this.updateGamePhase();
            this.updateTimer();
            this.updateDemoData();
            
            // Auto cycle through phases if enabled
            if (this.autoCycle) {
                this.cycleTimer++;
                if (this.cycleTimer > 100) { // 5 seconds at 20Hz
                    this.cycleTimer = 0;
                    const phases = ['disabled', 'auto', 'teleop', 'endgame1', 'endgame2'];
                    const currentIndex = phases.indexOf(this.currentPhase);
                    const nextIndex = (currentIndex + 1) % phases.length;
                    this.setPhase(phases[nextIndex]);
                }
            }
        }, 50); // 20Hz update rate
    }

    updateGamePhase() {
        // Only auto-update if not in demo mode or if connected to NT
        if (!this.demoMode && this.nt && this.nt.connected) {
            // Get robot state from NetworkTables
            const isEnabled = this.nt.getValue('/FMSInfo/IsEnabled', false);
            const isAuto = this.nt.getValue('/FMSInfo/IsAutonomous', false);
            const isTeleop = this.nt.getValue('/FMSInfo/IsTeleop', false);
            
            let newPhase = 'disabled';
            
            if (!isEnabled) {
                newPhase = 'disabled';
            } else if (isAuto) {
                newPhase = 'auto';
            } else if (isTeleop) {
                if (this.isCompetition) {
                    // Competition timing based on match time
                    if (this.matchTime > 30) {
                        newPhase = 'teleop';
                    } else if (this.matchTime > 15) {
                        newPhase = 'endgame1';
                    } else {
                        newPhase = 'endgame2';
                    }
                } else {
                    // Testing - no endgame phases
                    newPhase = 'teleop';
                }
            }
            
            if (newPhase !== this.currentPhase) {
                this.switchPhase(newPhase);
            }
        } else if (this.demoMode) {
            // In demo mode, auto-transition based on timer
            let newPhase = 'disabled';
            
            if (this.matchTime > 135) {
                newPhase = 'auto';
            } else if (this.matchTime > 30) {
                newPhase = 'teleop';
            } else if (this.matchTime > 15) {
                newPhase = 'endgame1';
            } else if (this.matchTime > 0) {
                newPhase = 'endgame2';
            } else {
                newPhase = 'disabled'; // Reset when timer reaches 0
            }
            
            if (newPhase !== this.currentPhase) {
                this.switchPhase(newPhase);
            }
        }
    }

    switchPhase(newPhase) {
        console.log(`Switching to phase: ${newPhase}`);
        
        const logo = document.querySelector('.logo');
        const previousPhase = this.currentPhase;
        
        // Trigger border blink on phase change - 4 cycles of 0.25s each
        if (newPhase !== this.currentPhase) {
            // Set the new phase color class first
            this.elements.alertBorder.className = 'alert-border ' + newPhase;
            logo.className = 'logo ' + newPhase;
            
            // Add blink class to trigger animation
            this.elements.alertBorder.classList.add('blink');
            logo.classList.add('blink');
            
            // Remove blink class after animation completes
            setTimeout(() => {
                this.elements.alertBorder.classList.remove('blink');
                logo.classList.remove('blink');
            }, 1000); // 4 blinks × 0.25s = 1s
        }
        
        this.currentPhase = newPhase;
        
        // Update game segment display
        const phaseText = newPhase.replace(/\d/, ' ').toUpperCase().replace('ENDGAME', 'ENDGAME PHASE');
        this.elements.gameSegment.textContent = phaseText;
        
        // Update alert border color class (already set above during blink)
        this.elements.alertBorder.className = 'alert-border ' + newPhase;
        
        // Hide all views
        Object.values(this.elements.views).forEach(view => {
            view.classList.remove('active', 'faded');
        });
        
        // Show appropriate view
        switch(newPhase) {
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
            case 'endgame1':
                this.elements.views.endgame1.classList.add('active');
                this.elements.mainTimer.classList.add('endgame-size');
                break;
            case 'endgame2':
                this.elements.views.endgame2.classList.add('active');
                this.elements.mainTimer.classList.add('endgame-size');
                break;
        }
    }

    updateTimer() {
        const minutes = Math.floor(this.matchTime / 60);
        const seconds = Math.floor(this.matchTime % 60);
        
        this.elements.timerMinutes.textContent = minutes.toString().padStart(2, '0');
        this.elements.timerSeconds.textContent = seconds.toString().padStart(2, '0');
        
        // Update auto timer if in auto - countdown from 15
        if (this.currentPhase === 'auto') {
            // Auto runs from 2:30 to 2:15, so display 15 to 0
            const autoTime = Math.max(0, Math.floor(this.matchTime - 135));
            this.elements.autoTimer.textContent = autoTime.toString();
        }
        
        // Only decrement timer if NOT in disabled mode
        if (this.currentPhase !== 'disabled') {
            if (this.demoMode || !this.nt.getValue('/FMSInfo/FMSAttached', false)) {
                this.matchTime = Math.max(0, this.matchTime - 0.05);
                
                // Reset for demo mode when timer reaches 0
                if (this.matchTime === 0 && this.demoMode) {
                    this.matchTime = 150; // Reset to 2:30
                    this.switchPhase('disabled');
                }
            }
        }
    }
}

// Initialize dashboard when page loads
let dashboard;
window.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing RoboDominators 5142 Dashboard');
    dashboard = new Dashboard();
});
