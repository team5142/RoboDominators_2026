// Operator Touchscreen Interface - Team 5142

const nt = new NT4Client('10.51.42.2:5810', 'OperatorInterface');

// UI Elements
const elements = {
    connectionLight: document.getElementById('connection-light'),
    enabledLight: document.getElementById('enabled-light'),
    messageBox: document.getElementById('message-box'),
    fullscreenBtn: document.getElementById('fullscreen-btn'),
    
    // Field elements - CHANGED to use circular click zones
    reefClickZones: document.querySelectorAll('.reef-click-zone:not(.disabled)'),
    processor: document.getElementById('processor'),
    coralStation: document.getElementById('coral-station'),
    
    // Action buttons
    orientField: document.getElementById('orient-field'),
    setReef17: document.getElementById('set-reef-17')
};
// State
let activeCommand = null;
let commandTimeout = null;

// Position name mapping
const POSITION_NAMES = {
    'BLUE_REEF_TAG_17': 'Blue Reef Tag 17',
    'BLUE_REEF_TAG_18': 'Blue Reef Tag 18',
    'BLUE_REEF_TAG_21': 'Blue Reef Tag 21',
    'BLUE_REEF_TAG_22': 'Blue Reef Tag 22',
    'BLUE_TAG_16': 'Blue Processor (Tag 16)',
    'BLUE_TAG_12': 'Blue Coral Station (Tag 12)'
};

// Initialize
function init() {
    setupNetworkTables();
    setupTouchHandlers();
    setupFullscreen();
    startHeartbeat();
    
    console.log('Operator Interface initialized');
}

// NetworkTables Setup
function setupNetworkTables() {
    nt.onConnect = () => {
        console.log('Connected to robot');
        elements.connectionLight.classList.add('connected');
    };
    
    nt.onDisconnect = () => {
        console.log('Disconnected from robot');
        elements.connectionLight.classList.remove('connected');
        elements.enabledLight.classList.remove('connected');
    };
    
    nt.subscribe('/RobotState/Enabled', (value) => {
        if (value) {
            elements.enabledLight.classList.add('connected');
        } else {
            elements.enabledLight.classList.remove('connected');
        }
    });
    
    nt.connect();
}

// Touch Event Handlers
function setupTouchHandlers() {
    // Reef click zones (circular buttons)
    elements.reefClickZones.forEach(zone => {
        zone.addEventListener('click', () => {
            const position = zone.dataset.position;
            const positionName = POSITION_NAMES[position];
            triggerDriveCommand(position, positionName);
        });
        
        // ADDED: Visual feedback on press - highlight the triangular reef face
        zone.addEventListener('pointerdown', () => {
            const reefId = zone.id.replace('click-', 'reef-'); // click-21 → reef-21
            const reefFace = document.getElementById(reefId);
            if (reefFace) {
                reefFace.classList.add('active');
            }
        });
        
        zone.addEventListener('pointerup', () => {
            const reefId = zone.id.replace('click-', 'reef-');
            const reefFace = document.getElementById(reefId);
            if (reefFace) {
                // Fade out after 300ms
                setTimeout(() => reefFace.classList.remove('active'), 300);
            }
        });
        
        zone.addEventListener('pointerleave', () => {
            const reefId = zone.id.replace('click-', 'reef-');
            const reefFace = document.getElementById(reefId);
            if (reefFace) {
                reefFace.classList.remove('active');
            }
        });
    });
    
    // Processor
    elements.processor.addEventListener('click', () => {
        triggerDriveCommand('BLUE_TAG_16', 'Blue Processor (Tag 16)');
    });
    
    // Coral Station
    elements.coralStation.addEventListener('click', () => {
        triggerDriveCommand('BLUE_TAG_12', 'Blue Coral Station (Tag 12)');
    });
    
    // Orient to Field button
    elements.orientField.addEventListener('click', () => {
        triggerAction('OrientToField', 'Orient to Field');
    });
    
    // Set Reef 17 Start Position button
    elements.setReef17.addEventListener('click', () => {
        triggerAction('SetReef17', 'Set Reef 17 Start Position');
    });
}

// Trigger drive-to-position command
function triggerDriveCommand(position, displayName) {
    console.log(`Drive to: ${displayName} (READ-ONLY MODE - not sent to robot)`);
    
    // DISABLED - operator is read-only
    // nt.publish(`/OperatorInterface/DriveToPosition/${position}`, 'boolean', true);
    // setTimeout(() => {
    //     nt.publish(`/OperatorInterface/DriveToPosition/${position}`, 'boolean', false);
    // }, 100);
    
    showMessage(`Drive to ${displayName} (READ-ONLY)`);
    setActiveElement(document.querySelector(`[data-position="${position}"]`));
}

// Trigger action command
function triggerAction(action, displayName) {
    console.log(`Action: ${displayName} (READ-ONLY MODE - not sent to robot)`);
    
    // DISABLED - operator is read-only
    // nt.publish(`/OperatorInterface/Action/${action}`, 'boolean', true);
    // setTimeout(() => {
    //     nt.publish(`/OperatorInterface/Action/${action}`, 'boolean', false);
    // }, 100);
    
    showMessage(`${displayName} (READ-ONLY)`);
    
    const button = action === 'OrientToField' ? elements.orientField : elements.setReef17;
    setActiveElement(button);
}

// Show message in message box
function showMessage(message) {
    elements.messageBox.textContent = message;
    
    if (commandTimeout) {
        clearTimeout(commandTimeout);
    }
    
    commandTimeout = setTimeout(() => {
        elements.messageBox.textContent = 'Ready';
    }, 3000);
}

// Set active visual feedback
function setActiveElement(element) {
    if (activeCommand) {
        activeCommand.classList.remove('active');
    }
    
    if (element) {
        element.classList.add('active');
        activeCommand = element;
        
        setTimeout(() => {
            if (activeCommand === element) {
                element.classList.remove('active');
                activeCommand = null;
            }
        }, 2000);
    }
}

// Fullscreen toggle
function setupFullscreen() {
    elements.fullscreenBtn.addEventListener('click', () => {
        if (!document.fullscreenElement) {
            document.documentElement.requestFullscreen();
        } else {
            document.exitFullscreen();
        }
    });
}

// Heartbeat
function startHeartbeat() {
    // DISABLED - operator is read-only, no heartbeat needed
    // setInterval(() => {
    //     nt.publish('/OperatorInterface/Heartbeat', 'boolean', true);
    // }, 500);
    console.log('Heartbeat disabled - operator in read-only mode');
}

window.addEventListener('DOMContentLoaded', init);