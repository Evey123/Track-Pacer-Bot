const statusText = document.getElementById('statusText');
const armText = document.getElementById('armText');
const serialText = document.getElementById('serialText');
const visionText = document.getElementById('visionText');
const errorText = document.getElementById('errorText');
const encoderText = document.getElementById('encoderText');
const distanceText = document.getElementById('distanceText');
const measuredSpeedText = document.getElementById('measuredSpeedText');
const targetPaceText = document.getElementById('targetPaceText');
const distanceSelect = document.getElementById('distanceSelect');
const timeInput = document.getElementById('timeInput');

async function postJson(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body ?? {}),
  });
  if (!res.ok) throw new Error(`Request failed: ${res.status}`);
  return res.json();
}

function renderStatus(status) {
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
  errorText.textContent = status.last_error || '';

  distanceSelect.value = String(status.target_distance_m);
  timeInput.value = Number(status.target_time_s ?? 0).toFixed(1);
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


document.getElementById('saveWorkoutBtn').addEventListener('click', async () => {
  const distance_m = Number(distanceSelect.value);
  const time_s = Number(timeInput.value);
  await action('/api/workout', { distance_m, time_s });
});

document.getElementById('armBtn').addEventListener('click', async () => action('/api/arm'));
document.getElementById('disarmBtn').addEventListener('click', async () => action('/api/disarm'));
document.getElementById('startBtn').addEventListener('click', async () => action('/api/start'));
document.getElementById('stopBtn').addEventListener('click', async () => action('/api/stop'));

document.getElementById('leftBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'left' }));
document.getElementById('centerBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'center' }));
document.getElementById('rightBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'right' }));
document.getElementById('visionBtn').addEventListener('click', async () => action('/api/manual_steer', { mode: 'vision' }));

refreshStatus();
setInterval(refreshStatus, 1000);
