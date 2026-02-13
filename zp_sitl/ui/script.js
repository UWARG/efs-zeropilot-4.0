let airspeedIndicator, attitude, altimeter, turnCoordinator, headingIndicator, vsi;
let ws;

$(document).ready(function() {
    // UI Event Listeners
    document.getElementById('startup-config').addEventListener('submit', (e) => {
        e.preventDefault();
        const config = {
            altitude: parseFloat(document.getElementById('init-altitude').value),
            speed: parseFloat(document.getElementById('init-speed').value),
            roll: parseFloat(document.getElementById('init-roll').value),
            pitch: parseFloat(document.getElementById('init-pitch').value),
            heading: parseFloat(document.getElementById('init-heading').value),
            throttle: parseFloat(document.getElementById('init-throttle').value),
            engine: document.getElementById('init-engine').checked
        };
        
        document.getElementById('startup-modal').classList.add('hidden');
        document.getElementById('sim-container').classList.remove('hidden');
        
        initSimulation(config);
    });

    startTelemViewer();
});

function initSimulation(config) {
    const imgDir = 'https://cdn.jsdelivr.net/gh/sebmatton/jQuery-Flight-Indicators@master/img/';
    const size = 200;

    // Initialize Flight Indicators
    attitude = $.flightIndicator('#attitude', 'attitude', { size, img_directory: imgDir });
    airspeedIndicator = $.flightIndicator('#airspeed-indicator', 'airspeed', { size, img_directory: imgDir });
    altimeter = $.flightIndicator('#altimeter', 'altimeter', { size, img_directory: imgDir });
    vsi = $.flightIndicator('#vsi', 'variometer', { size, img_directory: imgDir });
    turnCoordinator = $.flightIndicator('#turn-coordinator', 'turn_coordinator', { size, img_directory: imgDir });
    headingIndicator = $.flightIndicator('#heading-indicator', 'heading', { size, img_directory: imgDir });

    ws = new WebSocket('ws://localhost:8080/ws');
    
    ws.onopen = () => ws.send(JSON.stringify({type: 'init', config: config}));
    ws.onerror = (e) => console.error('WebSocket error:', e);
    
    const controls = {
        roll: document.getElementById('roll-ctrl'),
        pitch: document.getElementById('pitch-ctrl'),
        yaw: document.getElementById('yaw-ctrl'),
        throttle: document.getElementById('throttle-ctrl')
    };

    controls.throttle.value = config.throttle;
    document.getElementById('throttle-val').textContent = config.throttle;

    handleJoystick(ws, controls);

    document.getElementById('arm-btn').addEventListener('click', () => {
        if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type: 'arm'}));
    });

    // Manual Slider Input
    Object.entries(controls).forEach(([name, elem]) => {
        elem.addEventListener('input', () => {
            document.getElementById(`${name}-val`).textContent = elem.value;
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    type: 'control',
                    roll: parseInt(controls.roll.value),
                    pitch: parseInt(controls.pitch.value),
                    yaw: parseInt(controls.yaw.value),
                    throttle: parseInt(controls.throttle.value)
                }));
            }
        });
    });

    // Request State Updates
    setInterval(() => {
        if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({type: 'state'}));
    }, 100);

    ws.onmessage = (event) => {
        const state = JSON.parse(event.data);
        updateUI(state);
    };
}

function updateUI(state) {
    document.getElementById('roll').textContent = state.roll.toFixed(1) + '\u00B0';
    document.getElementById('pitch').textContent = state.pitch.toFixed(1) + '\u00B0';
    document.getElementById('yaw').textContent = state.yaw.toFixed(1) + '\u00B0';
    document.getElementById('altitude').textContent = state.altitude.toFixed(0) + ' ft';
    document.getElementById('airspeed').textContent = state.airspeed.toFixed(1) + ' kts';
    document.getElementById('rpm').textContent = state.rpm.toFixed(0);
    
    document.getElementById('roll-out').textContent = state.roll_output.toFixed(1);
    document.getElementById('pitch-out').textContent = state.pitch_output.toFixed(1);
    document.getElementById('yaw-out').textContent = state.yaw_output.toFixed(1);
    document.getElementById('throttle-out').textContent = state.throttle_output.toFixed(1);
    
    const armBtn = document.getElementById('arm-btn');
    armBtn.textContent = state.armed ? 'DISARM' : 'ARM';
    armBtn.className = state.armed ? 'arm-btn armed' : 'arm-btn disarmed';
    
    airspeedIndicator.setAirSpeed(state.airspeed);
    attitude.setRoll(-state.roll);
    attitude.setPitch(state.pitch);
    altimeter.setAltitude(state.altitude);
    turnCoordinator.setTurn(state.turn_rate);
    headingIndicator.setHeading(state.yaw);
    vsi.setVario(state.climb_rate / 1000);
}

function handleJoystick(ws, controls) {
    let joystickConnected = false;
    window.addEventListener("gamepadconnected", (e) => {
        joystickConnected = true;
        document.getElementById('joy-note').textContent = "Joystick Connected: " + e.gamepad.id;
        pollJoystick();
    });

    function pollJoystick() {
        if (!joystickConnected) return;
        const gp = navigator.getGamepads()[0];
        if (!gp) return;

        const applyDeadzone = (val) => (Math.abs(val) < 0.05 ? 0 : val);
        const norm = (val) => Math.round((val + 1) * 50);

        const joyData = {
            roll: norm(applyDeadzone(gp.axes[2])),
            pitch: norm(applyDeadzone(gp.axes[3])),
            yaw: norm(applyDeadzone(gp.axes[0])),
            throttle: norm(applyDeadzone(-gp.axes[1]))
        };

        Object.entries(joyData).forEach(([key, val]) => {
            controls[key].value = val;
            document.getElementById(`${key}-val`).textContent = val;
        });

        if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: 'control', ...joyData }));
        requestAnimationFrame(pollJoystick);
    }
}

function startTelemViewer() {
    const MAX_MESSAGES = 1000;

    const telemWs = new WebSocket('ws://localhost:8080/telem');
    const txDiv = document.getElementById('telem-tx');
    const rxDiv = document.getElementById('telem-rx');
    
    telemWs.onmessage = (event) => {
        const data = JSON.parse(event.data);
        const div = data.direction === 1 ? txDiv : rxDiv;
        const msgElem = document.createElement('div');
        msgElem.className = 'telemetry-message'; // Styles defined in CSS
        
        msgElem.innerHTML = data.decoded 
            ? `<strong>${data.type}</strong><br/><pre style="font-size: 11px; color: #aaa;">${data.decoded}</pre>` 
            : data.raw;
        
        div.insertBefore(msgElem, div.firstChild);
        if (div.children.length > MAX_MESSAGES) div.removeChild(div.lastChild);
    };
}
