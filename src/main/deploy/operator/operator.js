// Operator Touchscreen Interface - Team 5142

const nt = NetworkTableInstance.getDefault();

// ...existing code...

function triggerDriveCommand(position, displayName) {
    console.log(`Drive to: ${displayName}`);

    const topic = `/OperatorInterface/DriveToPosition/${position}`;
    nt.publish(topic, 'boolean', true);
    setTimeout(() => nt.publish(topic, 'boolean', false), 100);

    showMessage(`Drive to ${displayName}`);
    setActiveElement(document.querySelector(`[data-position="${position}"]`));
}

// ...existing code...