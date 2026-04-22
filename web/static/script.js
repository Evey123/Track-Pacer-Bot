const statusText = document.getElementById('statusText');
const armText = document.getElementById('armText');
const serialText = document.getElementById('serialText');
const visionText = document.getElementById('visionText');
const errorText = document.getElementById('errorText');
const encoderText = document.getElementById('encoderText');
const distanceText = document.getElementById('distanceText');
const measuredSpeedText = document.getElementById('measuredSpeedText');
const targetPaceText = document.getElementById('targetPaceText');
const motorPowerText = document.getElementById('motorPowerText');
const targetRotationsText = document.getElementById('targetRotationsText');
const distanceInput = document.getElementById('distanceInput');
const timeInput = document.getElementById('timeInput');

let plannerInitialized = false;

async function postJson(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body ?? {}),
  });
  if (!res.ok) throw new Error(`Request failed: ${res.status}`);
  return res.json();
}

function renderStatus(status, options = {}) {
  const syncPlanner = Boolean(options.syncPlanner);

  statusText.textContent = status.running ? 'RUNNING' : 'STOPPED';
  statusText.classList.toggle('running', status.running);
  statusText.classList.toggle('stopped', !status.running);

  armText.textContent = status.armed ? 'ARMED' : 'DISARMED';
  serialText.textContent = status.serial_ok ? 'OK' : 'NOT CONNECTED';
  visionText.textContent = status.vision_ok ? 'TRACK DETECTED' : 'NO TRACK';
  encoderText.textContent = status.encoder_count;
  distanceText.textContent = Number(status.measured_distance_m ?? 0).toFixed(2);
  measuredSpeedText.textContent = Number(status.measured_speed_mps ?? 0).toFixed(2);
  targetPaceText.textContent = Number(status.target_speed_mps ?? 0).toFixed(2);
  motorPowerText.textContent = Number(status.motor_power_percent ?? 0).toFixed(1);
  targetRotationsText.textContent = Number(status.target_rotations ?? 0).toFixed(2);
  errorText.textContent = status.last_error || '';

  if (!plannerInitialized || syncPlanner) {
    distanceInput.value = Number(status.target_distance_m ?? 0).toFixed(1);
    timeInput.value = Number(status.target_time_s ?? 0).toFixed(1);
    plannerInitialized = true;
  }
}

async function refreshStatus() {
  const res = await fetch('/api/status');
  const status = await res.json();
  renderStatus(status);
}

async function action(url, body) {
  try {
    const status = await postJson(url, body);
    renderStatus(status);
  } catch (err) {
    console.error(err);
    errorText.textContent = `Request failed: ${err.message}`;
  }
}

async function saveWorkout(syncPlanner = true) {
  const distance_m = Number(distanceInput.value);
  const time_s = Number(timeInput.value);

  try {
    const status = await postJson('/api/workout', { distance_m, time_s });
    renderStatus(status, { syncPlanner });
  } catch (err) {
    console.error(err);
    errorText.textContent = `Request failed: ${err.message}`;
  }
}

document.getElementById('saveWorkoutBtn').addEventListener('click', async () => {
  await saveWorkout(true);
});

document.getElementById('armBtn').addEventListener('click', async () => action('/api/arm'));
document.getElementById('disarmBtn').addEventListener('click', async () => action('/api/disarm'));
document.getElementById('startBtn').addEventListener('click', async () => action('/api/start'));
document.getElementById('stopBtn').addEventListener('click', async () => action('/api/stop'));

document.getElementById('leftBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'left' }));
document.getElementById('centerBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'center' }));
document.getElementById('rightBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'right' }));
document.getElementById('visionBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'vision' }));

document.querySelectorAll('.preset-btn').forEach((button) => {
  button.addEventListener('click', async () => {
    distanceInput.value = button.dataset.distance;
    await saveWorkout(true);
  });
});

refreshStatus();
setInterval(refreshStatus, 1000);
