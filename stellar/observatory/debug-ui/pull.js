const pullIntervalMs = 100;

let isReady = true;
setInterval(() => {
    if (isReady) {
        isReady = false;
        requestDebugData((event) => {
            const target = event.target;
            if (target.readyState != 4 || target.status != 200) {
                return;
            }

            updateUI(JSON.parse(target.response));
            isReady = true;
        });
    }
}, pullIntervalMs);

function requestDebugData(callback) {
    const xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = callback;
    xhttp.open('GET', '/debug-data', true);
    xhttp.send();
}

function updateUI(data) {
    if (!data) {
        return
    }

    updateTime(data.time);
    updateBattery(data.battery);
    updateMap(data.map);
    updateCameraFeed(data.cameraFeed);
    updateMotor(data.sensorMech.motor);
    updateSteering(data.sensorMech.steering);
    updateCPU(data.sensorElec.cpu);
    updateRAM(data.sensorElec.ram);
}

function updateTime(time) {
    document.getElementById('currentTime').innerText = getTimeString(time.current);
    document.getElementById('bestTime').innerText = `Best time: ${getTimeString(time.best)}`;
}

function updateBattery(battery) {
    document.getElementById('battery').innerText = `Battery: ${battery}%`;
}

function updateMap(map) {
    // TODO
}

function updateCameraFeed(cameraFeed) {
    if (cameraFeed) {
        document.getElementById('camera-feed').src = "data:image/jpg;base64," + cameraFeed;
    }
}

function updateMotor(motor) {
    document.getElementById('motor-value').innerText = `${motor} RPM`;
    document.getElementById('motor-meter').setAttribute('value', motor);
}

function updateSteering(steering) {
    document.getElementById('steering-value').innerText = `${steering}\u00B0`;
    document.getElementById('steering-meter').setAttribute('value', steering);
}

function updateCPU(cpu) {
    cpu.load.forEach((load, index) => {
        document.getElementById(`cpu${index}-value`).innerText = `${load}%`;
        document.getElementById(`cpu${index}-meter`).setAttribute('value', load);
    });
    document.getElementById('temp-value').innerText = `${cpu.temp}\u00B0`;
    document.getElementById('temp-meter').setAttribute('value', cpu.temp);
}

function updateRAM(ram) {
    document.getElementById('ram-value').innerText = `${ram}%`;
    document.getElementById('ram-meter').setAttribute('value', ram);
}

function getTimeString(data) {
    const date = new Date(data);
    const minutes = ('0' + date.getMinutes()).slice(-2);
    const seconds = ('0' + date.getSeconds()).slice(-2);
    const milliseconds = ('000' + date.getMilliseconds()).slice(-3);
    return `${minutes}:${seconds}.${milliseconds}`;
}
