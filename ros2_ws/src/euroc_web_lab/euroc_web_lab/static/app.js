const els = {
  statusDot: document.getElementById('statusDot'),
  statusText: document.getElementById('statusText'),
  statusSubtext: document.getElementById('statusSubtext'),
  topicName: document.getElementById('topicName'),
  encodingName: document.getElementById('encodingName'),
  displayMode: document.getElementById('displayMode'),
  frameCount: document.getElementById('frameCount'),
  rosTimestamp: document.getElementById('rosTimestamp'),
  captureBtn: document.getElementById('captureBtn'),
  gallery: document.getElementById('gallery'),
  undistortBtn: document.getElementById('undistortBtn'),
  rawImage: document.getElementById('rawImage'),
  undistortedImage: document.getElementById('undistortedImage'),
  selectedHint: document.getElementById('selectedHint'),
  calibrationBlock: document.getElementById('calibrationBlock'),
  bagSummary: document.getElementById('bagSummary'),
  bagRestartBtn: document.getElementById('bagRestartBtn'),
  bagSeekBtn: document.getElementById('bagSeekBtn'),
  bagProgressFill: document.getElementById('bagProgressFill'),
  bagProgressText: document.getElementById('bagProgressText'),
  bagSeekSlider: document.getElementById('bagSeekSlider'),
  bagSeekSec: document.getElementById('bagSeekSec'),
  bagState: document.getElementById('bagState'),
  bagRate: document.getElementById('bagRate'),
  bagLoop: document.getElementById('bagLoop'),
  bagMessages: document.getElementById('bagMessages'),
};

let selectedImage = null;
let savedImages = [];
let currentBag = null;
let seekDirty = false;
const MAX_CAPTURED_FRAMES = 6;

function cacheBust(url) {
  if (!url) return '';
  const joiner = url.includes('?') ? '&' : '?';
  return `${url}${joiner}t=${Date.now()}`;
}

function fmtTimestamp(ns) {
  if (!ns) return '-';
  return String(ns);
}

function fmtSeconds(value) {
  if (!Number.isFinite(value)) return '0.00';
  return value.toFixed(2);
}

function setBagUi(bag) {
  if (!bag) return;
  currentBag = bag;

  const position = Number(bag.position_sec || 0);
  const duration = Number(bag.duration_sec || 0);
  const progress = Math.max(0, Math.min(Number(bag.progress || 0), 1));
  const progressPct = progress * 100;
  const passNumber = Number(bag.loop_count || 0) + 1;
  const state = bag.running ? 'playing' : bag.return_code === null ? 'stopped' : `exited (${bag.return_code})`;

  els.bagProgressFill.style.width = `${progressPct}%`;
  els.bagProgressFill.parentElement.setAttribute('aria-valuenow', progressPct.toFixed(0));
  els.bagProgressText.textContent = `${fmtSeconds(position)} / ${fmtSeconds(duration)} s`;
  els.bagState.textContent = state;
  els.bagRate.textContent = `${bag.rate}x`;
  els.bagLoop.textContent = bag.loop_enabled ? `pass ${passNumber}` : 'off';
  els.bagMessages.textContent = bag.image_message_count || '-';
  els.bagSummary.textContent = bag.running
    ? `Playing ${fmtSeconds(position)} s of ${fmtSeconds(duration)} s from ${bag.path}`
    : `Bag player is not running. Last known position: ${fmtSeconds(position)} s.`;

  if (!seekDirty) {
    els.bagSeekSlider.value = duration > 0 ? Math.round(progress * 1000) : 0;
    els.bagSeekSec.max = duration > 0 ? duration.toFixed(2) : '';
    els.bagSeekSec.value = fmtSeconds(position);
  }
}

function compactCalibration(calibration, source) {
  if (!calibration) return 'Waiting for calibration...';
  const fx = calibration.K?.[0]?.[0];
  const fy = calibration.K?.[1]?.[1];
  const cx = calibration.K?.[0]?.[2];
  const cy = calibration.K?.[1]?.[2];
  return JSON.stringify({
    source: source || 'unknown',
    width: calibration.width,
    height: calibration.height,
    fx,
    fy,
    cx,
    cy,
    D: calibration.D,
    model: calibration.distortion_model,
  }, null, 2);
}

async function refreshStatus() {
  try {
    const response = await fetch('/api/status');
    const status = await response.json();
    els.topicName.textContent = status.image_topic;
    els.encodingName.textContent = status.encoding || '-';
    els.displayMode.textContent = status.display_mode || '-';
    els.statusSubtext.textContent = status.image_topic;
    els.frameCount.textContent = status.frame_count;
    els.rosTimestamp.textContent = fmtTimestamp(status.last_timestamp_ns);
    els.calibrationBlock.textContent = compactCalibration(status.calibration, status.calibration_source);
    setBagUi(status.bag);
    if (status.has_frame) {
      els.statusDot.className = 'dot dot-ok';
      els.statusText.textContent = 'Receiving ROS2 frames';
    } else {
      els.statusDot.className = 'dot dot-wait';
      els.statusText.textContent = 'Waiting for ROS2 frames';
    }
  } catch (err) {
    els.statusDot.className = 'dot dot-wait';
    els.statusText.textContent = 'Backend unavailable';
    els.statusSubtext.textContent = String(err);
  }
}

function selectedSeekOffset() {
  const duration = Number(currentBag?.duration_sec || 0);
  const fromInput = Number(els.bagSeekSec.value);
  if (Number.isFinite(fromInput)) {
    return Math.max(0, duration > 0 ? Math.min(fromInput, duration) : fromInput);
  }
  return duration * (Number(els.bagSeekSlider.value || 0) / 1000);
}

function setSeekInputFromSlider() {
  const duration = Number(currentBag?.duration_sec || 0);
  const next = duration * (Number(els.bagSeekSlider.value || 0) / 1000);
  els.bagSeekSec.value = fmtSeconds(next);
  seekDirty = true;
}

function setSeekSliderFromInput() {
  const duration = Number(currentBag?.duration_sec || 0);
  if (duration <= 0) return;
  const next = Math.max(0, Math.min(Number(els.bagSeekSec.value || 0), duration));
  els.bagSeekSlider.value = Math.round((next / duration) * 1000);
  seekDirty = true;
}

async function postBagControl(url, body = null) {
  els.bagRestartBtn.disabled = true;
  els.bagSeekBtn.disabled = true;
  try {
    const options = { method: 'POST' };
    if (body) {
      options.headers = { 'Content-Type': 'application/json' };
      options.body = JSON.stringify(body);
    }
    const response = await fetch(url, options);
    if (!response.ok) {
      const payload = await response.json();
      throw new Error(payload.detail || 'bag control failed');
    }
    seekDirty = false;
    setBagUi(await response.json());
    await refreshStatus();
  } catch (err) {
    alert(`Could not control bag playback: ${err}`);
  } finally {
    els.bagRestartBtn.disabled = false;
    els.bagSeekBtn.disabled = false;
  }
}

function restartBag() {
  seekDirty = false;
  postBagControl('/api/bag/restart');
}

function seekBag() {
  postBagControl('/api/bag/seek', { offset_sec: selectedSeekOffset() });
}

async function refreshGallery(selectNewest = false) {
  const response = await fetch('/api/saved');
  const payload = await response.json();
  savedImages = (payload.images || []).slice(0, MAX_CAPTURED_FRAMES);

  if (savedImages.length === 0) {
    els.gallery.className = 'gallery empty';
    els.gallery.innerHTML = 'No captured frames yet.';
    selectedImage = null;
    els.undistortBtn.disabled = true;
    return;
  }

  els.gallery.className = 'gallery';
  els.gallery.innerHTML = '';
  savedImages.forEach((item, index) => {
    const card = document.createElement('button');
    card.className = 'thumb';
    card.type = 'button';
    if (selectedImage?.image_id === item.image_id) card.classList.add('selected');

    const img = document.createElement('img');
    img.src = cacheBust(item.raw_url);
    img.alt = item.image_id;

    const label = document.createElement('span');
    label.textContent = `${item.image_id}${item.undistorted_url ? ' / undistorted' : ''}`;

    card.appendChild(img);
    card.appendChild(label);
    card.addEventListener('click', () => selectImage(item));
    els.gallery.appendChild(card);

    if (selectNewest && index === 0) selectImage(item);
  });
}

function selectImage(item) {
  selectedImage = item;
  els.rawImage.src = cacheBust(item.raw_url);
  els.undistortedImage.src = item.undistorted_url ? cacheBust(item.undistorted_url) : '';
  els.selectedHint.textContent = `Selected ${item.image_id}.`;
  els.undistortBtn.disabled = false;
  [...els.gallery.querySelectorAll('.thumb')].forEach((button) => {
    button.classList.toggle('selected', button.textContent.includes(item.image_id));
  });
}

async function captureFrame() {
  els.captureBtn.disabled = true;
  els.captureBtn.textContent = 'Capturing...';
  try {
    const response = await fetch('/api/capture', { method: 'POST' });
    if (!response.ok) {
      const payload = await response.json();
      throw new Error(payload.detail || 'capture failed');
    }
    await refreshGallery(true);
  } catch (err) {
    alert(`Could not capture frame: ${err}`);
  } finally {
    els.captureBtn.disabled = false;
    els.captureBtn.textContent = 'Capture Current Frame';
  }
}

async function undistortSelected() {
  if (!selectedImage) return;
  els.undistortBtn.disabled = true;
  els.undistortBtn.textContent = 'Undistorting...';
  try {
    const response = await fetch(`/api/undistort/${selectedImage.image_id}`, { method: 'POST' });
    if (!response.ok) {
      const payload = await response.json();
      throw new Error(payload.detail || 'undistortion failed');
    }
    const result = await response.json();
    els.undistortedImage.src = cacheBust(result.undistorted_url);
    await refreshGallery(false);
    const fresh = savedImages.find((img) => img.image_id === selectedImage.image_id) || selectedImage;
    selectImage(fresh);
  } catch (err) {
    alert(`Could not undistort frame: ${err}`);
  } finally {
    els.undistortBtn.disabled = false;
    els.undistortBtn.textContent = 'Apply Calibration / Undistort';
  }
}

els.captureBtn.addEventListener('click', captureFrame);
els.undistortBtn.addEventListener('click', undistortSelected);
els.bagRestartBtn.addEventListener('click', restartBag);
els.bagSeekBtn.addEventListener('click', seekBag);
els.bagSeekSlider.addEventListener('input', setSeekInputFromSlider);
els.bagSeekSec.addEventListener('input', setSeekSliderFromInput);

refreshStatus();
refreshGallery(false);
setInterval(refreshStatus, 1000);
