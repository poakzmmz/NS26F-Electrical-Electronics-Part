// Global state variables for Page 1 charts
let chartSpeed = null;
let chartRpm = null;
let chartGear = null;
let chartSteering = null;
let chartThrottleBrake = null;

// Global state variables for Page 2 charts
let diagChartThrottleBrake = null; // Stacked Top Sub-pane (Throttle & Brake)
let diagChartSteering = null;      // Stacked Bottom Sub-pane (Steering Angle)
let chartFL = null;
let chartFR = null;
let chartRL = null;
let chartRR = null;

// Global state variables for Page 4 charts
let chartCoolantOil = null;
let chartIntakeEcu = null;

// Global state variables for Page 3 IMU charts
let chartImuAccel = null;
let chartImuGyro = null;
let gpsImuCursorDragging = false;

// Zoom, Slicing & Scroll configurations
let globalData = [];
let currentStartSec = 0;
let currentEndSec = 30;
let totalDurationSec = 0;

// User specified boundary limits (Drag will be restricted inside this limit)
let limitStartSec = 0;
let limitEndSec = 0;

// Active downsampled dataset reference for easy cursor lookup
let activeSampledData = [];
let currentCursorIndex = 0;

// 화면용 다운샘플 인덱스 (전체 globalData 기준 위치) 및 그 시각값.
// 노이즈 필터는 100Hz 원본 전체에 먼저 적용된 뒤 이 인덱스로 추출됩니다.
let sampleIndices = [];
let sampleTimes = [];

// DOM Elements
const statusBadge = document.getElementById('file-status');
const statusText = document.getElementById('status-text');

// Cursor Realtime Value DOMs (Page 1)
const cursorSpeed = document.getElementById('cursor-speed');
const cursorSpeedRl = document.getElementById('cursor-speed-rl');
const cursorSpeedRr = document.getElementById('cursor-speed-rr');
const cursorRpm = document.getElementById('cursor-rpm');
const cursorGear = document.getElementById('cursor-gear');
const cursorSteering = document.getElementById('cursor-steering');
const cursorThrottle = document.getElementById('cursor-throttle');
const cursorBrake = document.getElementById('cursor-brake');

// Cursor Realtime Value DOMs (Page 2)
const diagCursorThrottle = document.getElementById('diag-cursor-throttle');
const diagCursorBrake = document.getElementById('diag-cursor-brake');
const diagCursorSteering = document.getElementById('diag-cursor-steering');
const cursorSusFl = document.getElementById('cursor-sus-fl');
const cursorSusFr = document.getElementById('cursor-sus-fr');
const cursorSusRl = document.getElementById('cursor-sus-rl');
const cursorSusRr = document.getElementById('cursor-sus-rr');

// Interactive Steering Wheel Graphic Widget DOM
const steeringWheelGraphic = document.getElementById('steering-wheel-graphic');
const gpsSteeringWheelGraphic = document.getElementById('gps-steering-wheel-graphic');
const gpsCursorSteering = document.getElementById('gps-cursor-steering');

// Diag Summary DOMs
const statMaxRpm = document.getElementById('stat-max-rpm');
const statMaxSpeed = document.getElementById('stat-max-speed');
const statMinBatt = document.getElementById('stat-min-batt');
const statDuration = document.getElementById('stat-duration');
const adcAlertBadge = document.getElementById('adc-alert-badge'); // (Safely handled in case of missing DOM)

// Zoom & Scroll Input DOMs
const inputStart = document.getElementById('input-start-time');
const inputEnd = document.getElementById('input-end-time');
const btnApply = document.getElementById('btn-apply-zoom');
const btnReset = document.getElementById('btn-reset-zoom');
const scrollBar = document.getElementById('timeline-scroll-bar');
const currentTimeVal = document.getElementById('current-time-val'); // 실시간 시점 시간 표시 DOM
const lblScrollType = document.getElementById('lbl-scroll-type');

// Tabs DOMs
const tabGeneral = document.getElementById('tab-general');
const tabDiagnostics = document.getElementById('tab-diagnostics');
const tabGps = document.getElementById('tab-gps');
const pageGeneral = document.getElementById('page-general');
const pageDiagnostics = document.getElementById('page-diagnostics');
const pageGps = document.getElementById('page-gps');
const tabTemperature = document.getElementById('tab-temperature');
const pageTemperature = document.getElementById('page-temperature');
const tabRealtime = document.getElementById('tab-realtime');
const pageRealtime = document.getElementById('page-realtime');
const tabHelp = document.getElementById('tab-help');
const pageHelp = document.getElementById('page-help');
const timelineNavigator = document.querySelector('.timeline-navigator');

// Temperature DOMs (Page 4)
const tempCursorCoolant = document.getElementById('temp-cursor-coolant');
const tempCursorOil = document.getElementById('temp-cursor-oil');
const tempCursorIat = document.getElementById('temp-cursor-iat');
const tempCursorEcu = document.getElementById('temp-cursor-ecu');
const tempMaxCoolant = document.getElementById('temp-max-coolant');
const tempMaxOil = document.getElementById('temp-max-oil');
const tempMaxIat = document.getElementById('temp-max-iat');
const tempMaxEcu = document.getElementById('temp-max-ecu');

// GPS DOMs
const cursorGpsCoords = document.getElementById('cursor-gps-coords');
const gpsCursorSpeed = document.getElementById('gps-cursor-speed');
const gpsCursorWheelSpeed = document.getElementById('gps-cursor-wheel-speed');
const gpsSpeedDelta = document.getElementById('gps-speed-delta');
const gpsCursorSats = document.getElementById('gps-cursor-sats');
const gpsCursorQual = document.getElementById('gps-cursor-qual');
const gpsCursorTime = document.getElementById('gps-cursor-time');
const imuAccelX = document.getElementById('imu-accel-x');
const imuAccelY = document.getElementById('imu-accel-y');
const imuRoll = document.getElementById('imu-roll');
const imuPitch = document.getElementById('imu-pitch');
const imuYaw = document.getElementById('imu-yaw');
const imuBattery = document.getElementById('imu-battery');
const imuAge = document.getElementById('imu-age');
const imuGDot = document.getElementById('imu-g-dot');
const gpsPlayToggle = document.getElementById('gps-play-toggle');
const gpsPlayRate = document.getElementById('gps-play-rate');
const gpsPlayTime = document.getElementById('gps-play-time');
const gpsImuLpf = document.getElementById('gps-imu-lpf');
const gpsLapSetLine = document.getElementById('gps-lap-set-line');
const gpsLapClear = document.getElementById('gps-lap-clear');
const gpsCheckpointAdd = document.getElementById('gps-checkpoint-add');
const gpsCheckpointClear = document.getElementById('gps-checkpoint-clear');
const gpsCheckpointCount = document.getElementById('gps-checkpoint-count');
const gpsSectorCard = document.getElementById('gps-sector-card');
const gpsSectorTable = document.getElementById('gps-sector-table');
const gpsSectorToggle = document.getElementById('gps-sector-toggle');
const gpsSectorOverlay = document.getElementById('gps-sector-overlay');
const gpsSectorOverlayTable = document.getElementById('gps-sector-overlay-table');
const gpsSectorOverlayClose = document.getElementById('gps-sector-overlay-close');
const gpsLapMinTime = document.getElementById('gps-lap-min-time');
const gpsLapToolbarStatus = document.getElementById('gps-lap-toolbar-status');
const gpsLapFixSummary = document.getElementById('gps-lap-fix-summary');
const gpsLapCount = document.getElementById('gps-lap-count');
const gpsLapBestTime = document.getElementById('gps-lap-best-time');
const gpsLapAverageDistance = document.getElementById('gps-lap-average-distance');
const gpsLapList = document.getElementById('gps-lap-list');
const gpsImuLpfFrequency = document.getElementById('gps-imu-lpf-frequency');
const gpsMapFullscreen = document.getElementById('gps-map-fullscreen');
const gpsLapMapLegend = document.getElementById('gps-lap-map-legend');
const gpsFullscreenPlayToggle = document.getElementById('gps-fullscreen-play-toggle');
const gpsFullscreenPlayRate = document.getElementById('gps-fullscreen-play-rate');
const gpsFullscreenTimeline = document.getElementById('gps-fullscreen-timeline');
const gpsFullscreenPlayTime = document.getElementById('gps-fullscreen-play-time');
const gpsFullscreenLapTimes = document.getElementById('gps-fullscreen-lap-times');
const gpsFullscreenSpeedValue = document.getElementById('gps-fullscreen-speed-value');
const gpsFullscreenDetailToggle = document.getElementById('gps-fullscreen-detail-toggle');
const gpsFullscreenDetail = document.getElementById('gps-fullscreen-detail');
const gpsGoProFile = document.getElementById('gps-gopro-file');
const gpsGoProOpen = document.querySelector('.gps-gopro-open');
const gpsGoProPanel = document.getElementById('gps-gopro-panel');
const gpsGoProVideo = document.getElementById('gps-gopro-video');
const gpsGoProCompareVideo = document.getElementById('gps-gopro-compare-video');
const gpsGoProPrimaryAudio = document.getElementById('gps-gopro-primary-audio');
const gpsGoProCompareAudio = document.getElementById('gps-gopro-compare-audio');
const gpsGoProPrimaryLabel = document.getElementById('gps-gopro-primary-label');
const gpsGoProCompareLabel = document.getElementById('gps-gopro-compare-label');
const gpsGoProPrimarySpeed = document.getElementById('gps-gopro-primary-speed');
const gpsGoProCompareSpeed = document.getElementById('gps-gopro-compare-speed');
const gpsGoProStatus = document.getElementById('gps-gopro-status');
const gpsGoProClose = document.getElementById('gps-gopro-close');
const gpsYouTubeOpen = document.getElementById('gps-youtube-open');
const gpsYouTubeDialog = document.getElementById('gps-youtube-dialog');
const gpsYouTubeForm = document.getElementById('gps-youtube-form');
const gpsYouTubeUrl = document.getElementById('gps-youtube-url');
const gpsYouTubeUrlClear = document.getElementById('gps-youtube-url-clear');
const gpsYouTubeCancel = document.getElementById('gps-youtube-cancel');
const gpsYouTubeCancelBottom = document.getElementById('gps-youtube-cancel-bottom');
const helpVideoTitleFile = document.getElementById('help-video-title-file');
const helpVideoTitleStatus = document.getElementById('help-video-title-status');
const helpVideoTitleOutput = document.getElementById('help-video-title-output');
const helpVideoDescriptionOutput = document.getElementById('help-video-description-output');
const helpVideoTitleCopy = document.getElementById('help-video-title-copy');
const helpVideoDescriptionCopy = document.getElementById('help-video-description-copy');
const gpsDetailSpeedValue = document.getElementById('gps-detail-speed-value');
const gpsDetailRpmValue = document.getElementById('gps-detail-rpm-value');
const gpsDetailGearValue = document.getElementById('gps-detail-gear-value');
const gpsDetailSteeringValue = document.getElementById('gps-detail-steering-value');
const gpsDetailPedalValue = document.getElementById('gps-detail-pedal-value');

// Theme Switcher DOM
const btnThemeToggle = document.getElementById('btn-theme-toggle');

// Modal Elements (보관함 제거 - 직접 업로드 방식)
const csvUploadInput = document.getElementById('csv-upload-input');
const loadedFileBadge = document.getElementById('loaded-file-badge');

// Helper to decode Hex safely
function parseHexOrInt(val) {
  if (val === undefined || val === null || val === '') return 0;
  const str = String(val).trim();
  if (str.startsWith('0x') || str.startsWith('0X')) {
    return parseInt(str, 16) || 0;
  }
  const parsedHex = parseInt(str, 16);
  if (!isNaN(parsedHex) && /[a-fA-F]/.test(str)) {
    return parsedHex;
  }
  return parseInt(str, 10) || parsedHex || 0;
}

// Telemetry_072 and later snapshots store one complete 8-byte CAN frame per
// column (can600_data ... can607_data), instead of one can_id/can_d0 ... row.
function decodePackedCanFrame(value) {
  if (value === undefined || value === null || value === '') {
    return new Array(8).fill(0);
  }

  let hex = String(value).trim().replace(/^0x/i, '').replace(/[^0-9a-f]/gi, '');
  if (!hex) return new Array(8).fill(0);
  hex = hex.padStart(16, '0').slice(-16);

  const bytes = [];
  for (let i = 0; i < 16; i += 2) {
    bytes.push(parseInt(hex.slice(i, i + 2), 16) || 0);
  }
  return bytes;
}

function packedCanFrameHasData(bytes) {
  return bytes.some(byte => byte !== 0);
}

// Keep the chart code compatible with both the legacy adc/wheel column names
// and the descriptive Telemetry_072 column names.
function normalizeTelemetryRow(row) {
  const mappings = {
    // Datalogger board connector / MCU scan order. ADC2 and ADC3 are unused:
    // ADC1=PC0 front brake, ADC4=PC3 FR, ADC5=PC4 steering, ADC6=PC5 FL.
    adc1_raw: 'front_brake_pressure_raw',
    adc4_raw: 'fr_potentiometer_raw',
    adc5_raw: 'steering_angle_raw',
    adc6_raw: 'fl_potentiometer_raw',
    wheel3_speed_centi_kmh: 'rr_wheel_speed_centi_kmh',
    wheel4_speed_centi_kmh: 'rl_wheel_speed_centi_kmh'
  };

  Object.entries(mappings).forEach(([legacyName, newName]) => {
    if ((row[legacyName] === undefined || row[legacyName] === null || row[legacyName] === '') &&
        row[newName] !== undefined) {
      row[legacyName] = row[newName];
    }
  });

  // Canonical names used by the new dashboard. Descriptive 072 columns take
  // priority, while legacy ADC logs remain supported as a fallback.
  row.front_brake_raw = row.front_brake_pressure_raw ?? row.adc1_raw;
  row.suspension_fl_raw = row.fl_potentiometer_raw ?? row.adc6_raw;
  row.suspension_fr_raw = row.fr_potentiometer_raw ?? row.adc4_raw;
  row.suspension_rl_raw = row.rl_potentiometer_raw;
  row.suspension_rr_raw = row.rr_potentiometer_raw;
  row.rear_brake_raw = row.rear_brake_pressure_raw;
  row.steering_raw = row.steering_angle_raw ?? row.adc5_raw;

  return row;
}

// Calibration functions for sensors
function getDecodedTps(rowTpsRaw) {
  return (rowTpsRaw || 0) * 0.5; // data[2] * 5U -> tps_x10 -> tps = data[2] * 0.5 (0~100%)
}

// Calibrated Brake Normalization
function getCalibratedBrake(rawValue) {
  const val = rawValue || 0;
  const percent = ((val - 390) / (1682 - 390)) * 100;
  return Math.max(0, Math.min(100, percent));
}

// Calibrated Steering
// 영점/배율/반전은 steering.js의 steeringCal에서 조정합니다 (핸들 그래픽 클릭).
// 기본값 {zeroRaw:998, degPerLsb:0.1, invert:false}는 기존 하드코딩 식과 동일:
//   (raw - 2048) * 0.1 + 105 = 0.1*raw - 99.8 = (raw - 998) * 0.1
function getCalibratedSteering(rawValue) {
  const cal = (typeof steeringCal !== 'undefined') ? steeringCal : { zeroRaw: 998, degPerLsb: 0.1, invert: false };
  const rawVal = (rawValue === undefined || rawValue === null || isNaN(rawValue)) ? cal.zeroRaw : rawValue;
  const deg = (rawVal - cal.zeroRaw) * 0.1;
  return cal.invert ? -deg : deg;
}

// GPS Map Global Variables
let gpsMap = null;
let gpsRouteLine = null;
let gpsCursorMarker = null;
let gpsGraphicLayer = null;
let gpsSatelliteLayer = null;
let currentGpsLayerMode = 'graphic'; // 'graphic' | 'satellite'
let gpsFinishLine = null;
let gpsFinishEndpointLayer = null;
let gpsLapCrossingLayer = null;
let gpsLapRouteLayer = null;
let gpsFinishPoints = [];
let gpsFinishPreviewLine = null;
let gpsFinishMarkers = [];
let gpsCheckpointLayer = null;
let gpsCheckpointDraftLayer = null;
let gpsCheckpointPreviewLine = null;
let gpsCheckpointSelectionActive = false;
let gpsCheckpointDraft = [];
let gpsCheckpoints = [];
let gpsLapPoints = [];
let gpsLapSelectionActive = false;
let gpsLapResults = [];
let gpsLapRouteLines = [];
let gpsSelectedLapIndex = -1;
let gpsSelectedLapIndices = [];
let gpsCompareMarkers = [];
let gpsDetailCharts = [];
let gpsDetailSourceData = null;
const GPS_LAP_COLORS = ['#00e5ff', '#ff3d9a', '#76ff03', '#ffca28', '#7c4dff', '#ff6d00', '#00e676', '#40c4ff'];
const GPS_FIXED_LINES_STORAGE_KEY = 'nssur_gps_fixed_lines_v1';
const CSV_GPS_UTC_OFFSET_SEC = 9 * 3600; // Logger gps_time is stored as Korea Standard Time (UTC+9).

// GPS + IMU synchronized playback state.
let gpsPlaybackActive = false;
let gpsPlaybackFrame = null;
let gpsPlaybackLastTimestamp = null;
let gpsPlaybackCursorSec = 0;
let gpsGoProObjectUrl = '';
let gpsGoProTelemetryStartSec = NaN;
let gpsGoProMatched = false;
let gpsGoProCompareLapIndex = -1;
let gpsGoProSourceType = '';
let gpsYouTubeApiPromise = null;
let gpsYouTubePrimaryPlayer = null;
let gpsYouTubeComparePlayer = null;
let gpsYouTubeVideoId = '';
let gpsGoProAudioSlot = '';
const gpsYouTubeLastSeekAt = new WeakMap();

function getVisibleYouTubePlayers() {
  const players = [gpsYouTubePrimaryPlayer];
  if (getGoProLapPair()) players.push(gpsYouTubeComparePlayer);
  return players.filter(Boolean);
}

function isYouTubeBuffering() {
  if (gpsGoProSourceType !== 'youtube') return false;
  const bufferingState = window.YT?.PlayerState?.BUFFERING;
  return getVisibleYouTubePlayers().some(player => player.getPlayerState?.() === bufferingState);
}

function isGoProPlaybackWaiting() {
  if (!gpsGoProMatched || !gpsPlaybackActive) return false;
  const pair = getGoProLapPair();
  const relativeTime = pair ? Number(scrollBar?.value) || 0 : NaN;
  if (gpsGoProSourceType === 'youtube') {
    const playingState = window.YT?.PlayerState?.PLAYING;
    const players = pair
      ? [
          relativeTime < gpsLapResults[pair.primaryIndex].duration ? gpsYouTubePrimaryPlayer : null,
          relativeTime < gpsLapResults[pair.compareIndex].duration ? gpsYouTubeComparePlayer : null
        ].filter(Boolean)
      : [gpsYouTubePrimaryPlayer].filter(Boolean);
    return players.some(player => player.getPlayerState?.() !== playingState);
  }
  const videos = pair
    ? [
        relativeTime < gpsLapResults[pair.primaryIndex].duration ? gpsGoProVideo : null,
        relativeTime < gpsLapResults[pair.compareIndex].duration ? gpsGoProCompareVideo : null
      ].filter(Boolean)
    : [gpsGoProVideo].filter(Boolean);
  return videos.some(video => video.paused || video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA);
}

function refreshYouTubeBufferingState() {
  const buffering = isYouTubeBuffering();
  gpsGoProPanel?.classList.toggle('youtube-buffering', buffering);
}

async function readMp4AtomHeader(file, offset) {
  if (offset + 8 > file.size) return null;
  const bytes = new Uint8Array(await file.slice(offset, Math.min(file.size, offset + 16)).arrayBuffer());
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let size = view.getUint32(0);
  const type = String.fromCharCode(...bytes.slice(4, 8));
  let headerSize = 8;
  if (size === 1 && bytes.length >= 16) {
    size = Number(view.getBigUint64(8));
    headerSize = 16;
  } else if (size === 0) size = file.size - offset;
  if (!Number.isFinite(size) || size < headerSize) return null;
  return { offset, size, type, headerSize };
}

async function extractMp4CreationDate(file) {
  let offset = 0;
  for (let count = 0; offset < file.size && count < 10000; count += 1) {
    const atom = await readMp4AtomHeader(file, offset);
    if (!atom) break;
    if (atom.type === 'moov') {
      const buffer = await file.slice(atom.offset, atom.offset + atom.size).arrayBuffer();
      const bytes = new Uint8Array(buffer);
      const view = new DataView(buffer);
      let child = atom.headerSize;
      while (child + 12 <= bytes.length) {
        let size = view.getUint32(child);
        const type = String.fromCharCode(...bytes.slice(child + 4, child + 8));
        let headerSize = 8;
        if (size === 1 && child + 16 <= bytes.length) {
          size = Number(view.getBigUint64(child + 8));
          headerSize = 16;
        }
        if (!Number.isFinite(size) || size < headerSize || child + size > bytes.length) break;
        if (type === 'mvhd') {
          const version = view.getUint8(child + headerSize);
          const creation = version === 1
            ? Number(view.getBigUint64(child + headerSize + 4))
            : view.getUint32(child + headerSize + 4);
          if (creation > 2082844800) return new Date((creation - 2082844800) * 1000);
        }
        child += size;
      }
      break;
    }
    offset += atom.size;
  }
  return null;
}

function extractYouTubeVideoId(value) {
  const raw = String(value || '').trim();
  if (/^[A-Za-z0-9_-]{11}$/.test(raw)) return raw;
  try {
    const url = new URL(/^https?:\/\//i.test(raw) ? raw : `https://${raw}`);
    const host = url.hostname.replace(/^www\./, '').toLowerCase();
    if (host === 'youtu.be') return url.pathname.split('/').filter(Boolean)[0] || '';
    if (host === 'youtube.com' || host === 'm.youtube.com' || host === 'music.youtube.com') {
      if (url.pathname === '/watch') return url.searchParams.get('v') || '';
      const parts = url.pathname.split('/').filter(Boolean);
      if (['embed', 'shorts', 'live'].includes(parts[0])) return parts[1] || '';
    }
  } catch (_) {
    return '';
  }
  return '';
}

function parseYouTubeKstStartDate(title) {
  const match = String(title || '').match(/(?:^|[^0-9])(\d{4})[-_.](\d{2})[-_.](\d{2})[_\s-]+(\d{2})[-:.](\d{2})[-:.](\d{2})(?:[.,](\d{1,3}))?[_\s-]*KST(?:[^A-Z]|$)/i);
  if (!match) return null;
  const [, yearText, monthText, dayText, hourText, minuteText, secondText, millisText = '0'] = match;
  const year = Number(yearText);
  const month = Number(monthText);
  const day = Number(dayText);
  const hour = Number(hourText);
  const minute = Number(minuteText);
  const second = Number(secondText);
  const millis = Number(millisText.padEnd(3, '0'));
  if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) return null;
  return new Date(Date.UTC(year, month - 1, day, hour - 9, minute, second, millis));
}

function getKstDateParts(date) {
  const parts = new Intl.DateTimeFormat('en-CA', {
    timeZone: 'Asia/Seoul',
    year: 'numeric', month: '2-digit', day: '2-digit',
    hour: '2-digit', minute: '2-digit', second: '2-digit',
    hourCycle: 'h23'
  }).formatToParts(date);
  return Object.fromEntries(parts.map(part => [part.type, part.value]));
}

function makeYouTubeUploadMetadata(file, creationDate) {
  const { year, month, day, hour, minute, second } = getKstDateParts(creationDate);
  const sourceName = String(file?.name || 'GOPRO').replace(/\.mp4$/i, '');
  const safeName = sourceName.replace(/[^A-Za-z0-9_-]+/g, '-').replace(/^-+|-+$/g, '') || 'GOPRO';
  const startText = `${year}-${month}-${day} ${hour}:${minute}:${second} KST`;
  return {
    title: `NS26F_${year}-${month}-${day}_${hour}-${minute}-${second}_KST_${safeName}`.slice(0, 100),
    description: `NSSUR Telemetry 동기화 정보\n영상 시작: ${startText}\n원본 파일: ${file.name}\n\n※ 설명은 기록용이며, 사이트 동기화는 YouTube 제목의 시작 시각을 사용합니다.`,
    startText
  };
}

async function copyHelpVideoText(value, button, defaultLabel) {
  if (!value) return;
  try {
    await navigator.clipboard.writeText(value);
    button.textContent = '복사 완료';
    window.setTimeout(() => { button.textContent = defaultLabel; }, 1400);
  } catch (_) {
    const field = button.closest('div')?.querySelector('input, textarea');
    field?.select();
    document.execCommand('copy');
    button.textContent = '복사 완료';
    window.setTimeout(() => { button.textContent = defaultLabel; }, 1400);
  }
}

function loadYouTubeIframeApi() {
  if (window.YT?.Player) return Promise.resolve(window.YT);
  if (gpsYouTubeApiPromise) return gpsYouTubeApiPromise;
  gpsYouTubeApiPromise = new Promise((resolve, reject) => {
    const previousReady = window.onYouTubeIframeAPIReady;
    window.onYouTubeIframeAPIReady = () => {
      if (typeof previousReady === 'function') previousReady();
      resolve(window.YT);
    };
    let script = document.querySelector('script[src="https://www.youtube.com/iframe_api"]');
    if (!script) {
      script = document.createElement('script');
      script.src = 'https://www.youtube.com/iframe_api';
      script.async = true;
      document.head.appendChild(script);
    }
    script.addEventListener('error', () => reject(new Error('YouTube 플레이어를 불러오지 못했습니다. 인터넷 연결을 확인하세요.')), { once: true });
    window.setTimeout(() => {
      if (!window.YT?.Player) reject(new Error('YouTube 플레이어 연결 시간이 초과되었습니다.'));
    }, 15000);
  });
  return gpsYouTubeApiPromise;
}

function ensureYouTubeMount(id, slotSelector) {
  let mount = document.getElementById(id);
  if (mount) return mount;
  const slot = document.querySelector(slotSelector);
  if (!slot) return null;
  mount = document.createElement('div');
  mount.id = id;
  mount.className = 'gps-youtube-player';
  slot.appendChild(mount);
  return mount;
}

function createYouTubePlayer(mount, videoId) {
  return new Promise((resolve, reject) => {
    if (!mount || !window.YT?.Player) {
      reject(new Error('YouTube 플레이어 영역을 준비하지 못했습니다.'));
      return;
    }
    let settled = false;
    const player = new window.YT.Player(mount, {
      videoId,
      width: '100%',
      height: '100%',
      playerVars: {
        autoplay: 0,
        controls: 0,
        disablekb: 1,
        fs: 0,
        playsinline: 1,
        rel: 0
      },
      events: {
        onReady: event => {
          event.target.getIframe?.().classList.add('gps-youtube-player');
          event.target.mute();
          settled = true;
          resolve(event.target);
        },
        onStateChange: event => {
          if (event.data === window.YT?.PlayerState?.PLAYING &&
              (!gpsPlaybackActive || gpsGoProSourceType !== 'youtube')) {
            event.target.pauseVideo?.();
          }
          requestAnimationFrame(refreshYouTubeBufferingState);
        },
        onError: event => {
          if (!settled) reject(new Error(`YouTube 영상을 재생할 수 없습니다. 오류 코드 ${event.data}`));
        }
      }
    });
  });
}

async function waitForYouTubeMetadata(player) {
  for (let count = 0; count < 40; count += 1) {
    const duration = Number(player?.getDuration?.());
    const title = String(player?.getVideoData?.()?.title || '').trim();
    if (duration > 0 && title) return { duration, title };
    await new Promise(resolve => window.setTimeout(resolve, 125));
  }
  return {
    duration: Number(player?.getDuration?.()),
    title: String(player?.getVideoData?.()?.title || '').trim()
  };
}

function destroyYouTubePlayers() {
  [gpsYouTubePrimaryPlayer, gpsYouTubeComparePlayer].forEach(player => {
    try { player?.destroy?.(); } catch (_) { /* 이미 제거된 플레이어 */ }
  });
  gpsYouTubePrimaryPlayer = null;
  gpsYouTubeComparePlayer = null;
  gpsYouTubeVideoId = '';
  gpsGoProPanel?.classList.remove('youtube-buffering');
  ['gps-youtube-player', 'gps-youtube-compare-player'].forEach(id => {
    const element = document.getElementById(id);
    if (element?.tagName === 'IFRAME') element.remove();
  });
  ensureYouTubeMount('gps-youtube-player', '.gps-gopro-primary-slot');
  ensureYouTubeMount('gps-youtube-compare-player', '.gps-gopro-compare-slot');
}

function getCsvGpsClockRange() {
  let first = null;
  let last = null;
  let previousClock = NaN;
  let dayOffset = 0;
  for (const row of globalData) {
    const rawClock = parseGpsClockSeconds(row.gps_time);
    if (!Number.isFinite(rawClock)) continue;
    if (Number.isFinite(previousClock) && rawClock + dayOffset < previousClock - 43200) dayOffset += 86400;
    const clock = rawClock + dayOffset;
    const point = { clock, telemetry: Number(row.time_sec) };
    if (!Number.isFinite(point.telemetry)) continue;
    if (!first) first = point;
    last = point;
    previousClock = clock;
  }
  return first && last ? { first, last } : null;
}

function matchGoProToCsv(creationDate, duration) {
  const range = getCsvGpsClockRange();
  if (!range || !Number.isFinite(duration)) return null;
  const rawStartUtc = creationDate.getUTCHours() * 3600 + creationDate.getUTCMinutes() * 60 + creationDate.getUTCSeconds() + creationDate.getUTCMilliseconds() / 1000;
  const rawStart = rawStartUtc + CSV_GPS_UTC_OFFSET_SEC;
  let best = null;
  for (let day = -1; day <= 1; day += 1) {
    const videoStart = rawStart + day * 86400;
    const overlap = Math.min(range.last.clock, videoStart + duration) - Math.max(range.first.clock, videoStart);
    if (!best || overlap > best.overlap) best = { videoStart, overlap, range };
  }
  if (!best || best.overlap <= 0) return best ? { ...best, matched: false } : null;
  return {
    matched: true,
    telemetryStart: best.range.first.telemetry + (best.videoStart - best.range.first.clock),
    overlap: best.overlap,
    videoStartClock: best.videoStart
  };
}

function syncOneGoProVideo(video, targetTime, force, rate, holdAtFinish = false) {
  const videoTime = targetTime - gpsGoProTelemetryStartSec;
  if (gpsGoProSourceType === 'youtube') {
    const player = video === gpsGoProCompareVideo ? gpsYouTubeComparePlayer : gpsYouTubePrimaryPlayer;
    const duration = Number(player?.getDuration?.());
    if (!player || !Number.isFinite(duration) || duration <= 0) return;
    if (videoTime < 0 || videoTime > duration) {
      player.pauseVideo?.();
      return;
    }
    const currentTime = Number(player.getCurrentTime?.());
    const drift = currentTime - videoTime;
    const now = performance.now();
    const lastSeekAt = gpsYouTubeLastSeekAt.get(player) || 0;
    const needsDriftCorrection = Math.abs(drift) > 0.75 && now - lastSeekAt > 750;
    if (force || !gpsPlaybackActive || !Number.isFinite(currentTime) || needsDriftCorrection) {
      player.seekTo?.(videoTime, true);
      gpsYouTubeLastSeekAt.set(player, now);
    }
    player.setPlaybackRate?.(rate);
    const state = player.getPlayerState?.();
    if (holdAtFinish) {
      if (force || !Number.isFinite(currentTime) || Math.abs(drift) > 0.08) player.seekTo?.(videoTime, true);
      player.pauseVideo?.();
    } else {
      const canStart = [
        window.YT?.PlayerState?.UNSTARTED,
        window.YT?.PlayerState?.ENDED,
        window.YT?.PlayerState?.PAUSED,
        window.YT?.PlayerState?.CUED
      ].includes(state);
      if (gpsPlaybackActive && canStart) player.playVideo?.();
      if (!gpsPlaybackActive) player.pauseVideo?.();
    }
    return;
  }
  if (!video || !Number.isFinite(video.duration)) return;
  if (videoTime < 0 || videoTime > video.duration) {
    video.pause();
    return;
  }
  const drift = video.currentTime - videoTime;
  if (force || !gpsPlaybackActive || Math.abs(drift) > 1.0 || (holdAtFinish && Math.abs(drift) > 0.03)) {
    video.currentTime = videoTime;
  }
  video.playbackRate = rate;
  if (holdAtFinish) video.pause();
  else {
    if (gpsPlaybackActive && video.paused) video.play().catch(() => {});
    if (!gpsPlaybackActive && !video.paused) video.pause();
  }
}

function getGoProLapPair() {
  if (gpsSelectedLapIndices.length < 2) return null;
  const primaryIndex = gpsSelectedLapIndices.reduce((best, index) =>
    gpsLapResults[index].duration < gpsLapResults[best].duration ? index : best);
  const compareIndex = gpsSelectedLapIndices.includes(gpsGoProCompareLapIndex) && gpsGoProCompareLapIndex !== primaryIndex
    ? gpsGoProCompareLapIndex
    : gpsSelectedLapIndices.find(index => index !== primaryIndex);
  return { primaryIndex, compareIndex };
}

function updateGoProComparisonLayout() {
  const pair = getGoProLapPair();
  gpsGoProPanel?.classList.toggle('comparing', Boolean(pair));
  if (!pair) {
    if (gpsGoProAudioSlot === 'compare') gpsGoProAudioSlot = 'primary';
    if (gpsGoProPrimaryLabel) gpsGoProPrimaryLabel.textContent = '';
    if (gpsGoProSourceType === 'youtube') gpsYouTubeComparePlayer?.pauseVideo?.();
    else gpsGoProCompareVideo?.pause();
    applyGoProAudioSelection();
    return;
  }
  const primary = gpsLapResults[pair.primaryIndex];
  const compare = gpsLapResults[pair.compareIndex];
  if (gpsGoProPrimaryLabel) {
    gpsGoProPrimaryLabel.textContent = `기준 · LAP ${primary.number} · ${formatLapTime(primary.duration)}`;
    gpsGoProPrimaryLabel.parentElement?.style.setProperty('--lap-color', GPS_LAP_COLORS[pair.primaryIndex % GPS_LAP_COLORS.length]);
  }
  if (gpsGoProCompareLabel) {
    gpsGoProCompareLabel.textContent = `비교 · LAP ${compare.number} · +${(compare.duration - primary.duration).toFixed(3)}초`;
    gpsGoProCompareLabel.parentElement?.style.setProperty('--lap-color', GPS_LAP_COLORS[pair.compareIndex % GPS_LAP_COLORS.length]);
  }
  applyGoProAudioSelection();
}

function gpsSpeedAtTelemetryTime(targetTime) {
  const index = globalData.length ? findGlobalIndexAtTime(targetTime) : -1;
  return index >= 0 ? Number(globalData[index].gps_speed_kmh) || 0 : 0;
}

function setGoProSlotSpeed(element, speed) {
  if (element?.firstChild) element.firstChild.nodeValue = `${speed.toFixed(1)} `;
}

function applyGoProAudioSelection() {
  const primaryEnabled = gpsGoProAudioSlot === 'primary';
  const compareEnabled = gpsGoProAudioSlot === 'compare' && Boolean(getGoProLapPair());
  if (gpsGoProVideo) gpsGoProVideo.muted = !primaryEnabled;
  if (gpsGoProCompareVideo) gpsGoProCompareVideo.muted = !compareEnabled;
  if (primaryEnabled) gpsYouTubePrimaryPlayer?.unMute?.();
  else gpsYouTubePrimaryPlayer?.mute?.();
  if (compareEnabled) gpsYouTubeComparePlayer?.unMute?.();
  else gpsYouTubeComparePlayer?.mute?.();
  gpsGoProPrimaryAudio?.setAttribute('aria-pressed', String(primaryEnabled));
  gpsGoProCompareAudio?.setAttribute('aria-pressed', String(compareEnabled));
  gpsGoProPrimaryAudio?.setAttribute('aria-label', primaryEnabled ? '기준 영상 소리 끄기' : '기준 영상 소리 켜기');
  gpsGoProCompareAudio?.setAttribute('aria-label', compareEnabled ? '비교 영상 소리 끄기' : '비교 영상 소리 켜기');
}

function toggleGoProAudio(slot) {
  gpsGoProAudioSlot = gpsGoProAudioSlot === slot ? '' : slot;
  applyGoProAudioSelection();
}

function syncGoProVideo(targetTime, force = false) {
  if (!gpsGoProMatched || !gpsGoProVideo) return;
  const rate = Number(gpsPlayRate?.value) || 1;
  const pair = getGoProLapPair();
  if (pair) {
    const timelineLap = gpsLapResults[gpsSelectedLapIndices[0]];
    const relativeTime = Math.max(0, targetTime - timelineLap.startTime);
    const primary = gpsLapResults[pair.primaryIndex];
    const compare = gpsLapResults[pair.compareIndex];
    const primaryTime = primary.startTime + Math.min(relativeTime, primary.duration);
    const compareTime = compare.startTime + Math.min(relativeTime, compare.duration);
    const primaryArrived = relativeTime >= primary.duration;
    const compareArrived = relativeTime >= compare.duration;
    syncOneGoProVideo(gpsGoProVideo, primaryTime, force, rate, primaryArrived);
    syncOneGoProVideo(gpsGoProCompareVideo, compareTime, force, rate, compareArrived);
    gpsGoProPanel?.classList.toggle('primary-arrived', primaryArrived);
    gpsGoProPanel?.classList.toggle('compare-arrived', compareArrived);
    setGoProSlotSpeed(gpsGoProPrimarySpeed, gpsSpeedAtTelemetryTime(primaryTime));
    setGoProSlotSpeed(gpsGoProCompareSpeed, gpsSpeedAtTelemetryTime(compareTime));
  } else {
    gpsGoProPanel?.classList.remove('primary-arrived', 'compare-arrived');
    syncOneGoProVideo(gpsGoProVideo, targetTime, force, rate);
    if (gpsGoProSourceType === 'youtube') gpsYouTubeComparePlayer?.pauseVideo?.();
    else gpsGoProCompareVideo?.pause();
  }
}

function getGoProTargetTelemetryTime(cursorTime) {
  if (gpsSelectedLapIndices.length > 1) {
    const primaryLap = gpsLapResults[gpsSelectedLapIndices[0]];
    if (primaryLap) return primaryLap.startTime + Math.min(cursorTime, primaryLap.duration);
  }
  return cursorTime;
}

function closeGoProVideo() {
  if (gpsGoProSourceType === 'youtube') {
    gpsYouTubePrimaryPlayer?.pauseVideo?.();
    gpsYouTubeComparePlayer?.pauseVideo?.();
  } else {
    gpsGoProVideo?.pause();
    gpsGoProCompareVideo?.pause();
  }
  if (gpsGoProVideo) gpsGoProVideo.removeAttribute('src');
  if (gpsGoProCompareVideo) gpsGoProCompareVideo.removeAttribute('src');
  if (gpsGoProObjectUrl) URL.revokeObjectURL(gpsGoProObjectUrl);
  destroyYouTubePlayers();
  gpsGoProObjectUrl = '';
  gpsGoProAudioSlot = '';
  applyGoProAudioSelection();
  gpsGoProSourceType = '';
  gpsGoProMatched = false;
  gpsGoProCompareLapIndex = -1;
  gpsGoProTelemetryStartSec = NaN;
  const stage = gpsGoProPanel?.closest('.gps-map-stage');
  stage?.classList.remove('gps-video-loaded');
  if (gpsGoProPanel) {
    gpsGoProPanel.hidden = true;
    gpsGoProPanel.classList.remove('youtube-source', 'youtube-buffering', 'primary-arrived', 'compare-arrived');
  }
  if (gpsGoProFile) gpsGoProFile.value = '';
  setTimeout(() => {
    gpsMap?.invalidateSize();
    refitGpsMapToCurrentLapView();
  }, 80);
}

// NMEA coordinate converter helper
function convertNmeaToDecimal(val, isLongitude = false) {
  if (val === undefined || val === null) return null;
  const strVal = val.toString().trim();
  if (strVal === "" || isNaN(strVal)) return null;
  
  const num = parseFloat(strVal);
  if (num === 0) return null;

  // New logs may already contain signed decimal-degree coordinates.
  const decimalLimit = isLongitude ? 180 : 90;
  if (Math.abs(num) <= decimalLimit) return num;
  
  const limit = isLongitude ? 3 : 2;
  if (strVal.length < limit + 2) return null;
  
  const degreesStr = strVal.substring(0, limit);
  const minutesStr = strVal.substring(limit);
  
  const degrees = parseFloat(degreesStr);
  const minutes = parseFloat(minutesStr);
  
  if (isNaN(degrees) || isNaN(minutes)) return null;
  return degrees + (minutes / 60.0);
}

function formatLapTime(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return '--:--.---';
  const totalMs = Math.round(seconds * 1000);
  const minutes = Math.floor(totalMs / 60000);
  const secs = Math.floor((totalMs % 60000) / 1000);
  const millis = totalMs % 1000;
  return `${minutes}:${String(secs).padStart(2, '0')}.${String(millis).padStart(3, '0')}`;
}

function formatKoreanDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return '--';
  const minutes = Math.floor(seconds / 60);
  const remaining = seconds - minutes * 60;
  return `${minutes}분 ${remaining.toFixed(1).padStart(4, '0')}초`;
}

function parseGpsClockSeconds(value) {
  if (value === undefined || value === null) return NaN;
  const text = String(value).trim();
  if (!text || text === '00:00:00.00') return NaN;
  const colon = text.match(/^(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)$/);
  if (colon) return Number(colon[1]) * 3600 + Number(colon[2]) * 60 + Number(colon[3]);
  const compact = text.match(/^(\d{2})(\d{2})(\d{2}(?:\.\d+)?)$/);
  if (compact) return Number(compact[1]) * 3600 + Number(compact[2]) * 60 + Number(compact[3]);
  return NaN;
}

function formatGpsClock(seconds) {
  if (!Number.isFinite(seconds)) return '--:--:--.--';
  const daySeconds = ((seconds % 86400) + 86400) % 86400;
  const hours = Math.floor(daySeconds / 3600);
  const minutes = Math.floor((daySeconds % 3600) / 60);
  const secs = daySeconds % 60;
  return `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${secs.toFixed(2).padStart(5, '0')}`;
}

function gpsClockAtTelemetryTime(targetTime, nearbyIndex = -1) {
  if (!globalData.length || !Number.isFinite(targetTime)) return NaN;
  let index = nearbyIndex >= 0 ? nearbyIndex : findGlobalIndexAtTime(targetTime);
  index = Math.max(0, Math.min(globalData.length - 1, index));
  for (let distance = 0; distance < globalData.length; distance += 1) {
    for (const candidateIndex of distance ? [index - distance, index + distance] : [index]) {
      if (candidateIndex < 0 || candidateIndex >= globalData.length) continue;
      const row = globalData[candidateIndex];
      const clock = parseGpsClockSeconds(row.gps_time);
      if (Number.isFinite(clock) && Number.isFinite(row.time_sec)) {
        return clock + (targetTime - row.time_sec);
      }
    }
  }
  return NaN;
}

function setGpsLapStatus(text, className = '') {
  if (!gpsLapToolbarStatus) return;
  gpsLapToolbarStatus.textContent = text;
  gpsLapToolbarStatus.className = className;
}

function latLonToLocalMeters(point, origin) {
  const rad = Math.PI / 180;
  return {
    x: (point.lon - origin.lon) * 111320 * Math.cos(origin.lat * rad),
    y: (point.lat - origin.lat) * 110540
  };
}

function cross2(a, b) {
  return a.x * b.y - a.y * b.x;
}

function distanceMeters(a, b) {
  const origin = { lat: (a.lat + b.lat) * 0.5, lon: (a.lon + b.lon) * 0.5 };
  const pa = latLonToLocalMeters(a, origin);
  const pb = latLonToLocalMeters(b, origin);
  return Math.hypot(pb.x - pa.x, pb.y - pa.y);
}

function formatGpsLapDistance(distance) {
  return Number.isFinite(distance) ? `${distance.toLocaleString('ko-KR', { minimumFractionDigits: 1, maximumFractionDigits: 1 })} m` : '--.- m';
}

function calculateGpsLapDistance(lap) {
  const points = [
    { lat: lap.startLat, lon: lap.startLon, time: lap.startTime },
    ...gpsLapPoints.filter(point => point.time > lap.startTime && point.time < lap.endTime),
    { lat: lap.endLat, lon: lap.endLon, time: lap.endTime }
  ];
  let total = 0;
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1];
    const current = points[index];
    const elapsed = current.time - previous.time;
    const segment = distanceMeters(previous, current);
    if (!(elapsed > 0) || !Number.isFinite(segment)) continue;
    // GPS 점프와 장시간 수신 공백은 거리 합계에서 제외합니다.
    if (elapsed > 3 || segment / elapsed > 120) continue;
    total += segment;
  }
  return total > 0 ? total : NaN;
}

function buildGpsLapPoints(data) {
  const points = [];
  let lastFixCounter = null;
  let lastFallbackKey = null;
  let previousGpsClock = null;
  let gpsDayOffset = 0;

  data.forEach(row => {
    const lat = convertNmeaToDecimal(row.gps_lat, false);
    const lon = convertNmeaToDecimal(row.gps_lon, true);
    const time = Number(row.time_sec);
    const quality = Number.parseInt(row.gps_qual, 10);
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || !Number.isFinite(time) || quality <= 0) return;

    const fixCounter = Number.parseInt(row.gps_fix_update_count, 10);
    if (Number.isFinite(fixCounter) && fixCounter > 0) {
      if (fixCounter === lastFixCounter) return;
      lastFixCounter = fixCounter;
    } else {
      const fallbackKey = `${row.gps_time || ''}|${lat.toFixed(8)}|${lon.toFixed(8)}`;
      if (fallbackKey === lastFallbackKey) return;
      lastFallbackKey = fallbackKey;
    }

    const rawGpsClock = parseGpsClockSeconds(row.gps_time);
    if (Number.isFinite(rawGpsClock) && previousGpsClock !== null && rawGpsClock < previousGpsClock - 43200) {
      gpsDayOffset += 86400;
    }
    if (Number.isFinite(rawGpsClock)) previousGpsClock = rawGpsClock;

    points.push({
      lat,
      lon,
      time,
      gpsTime: Number.isFinite(rawGpsClock) ? rawGpsClock + gpsDayOffset : NaN,
      speed: Number(row.gps_speed_kmh) || 0,
      quality,
      sats: Number.parseInt(row.gps_sat, 10) || 0
    });
  });

  return points;
}

function drawGpsFinishLine() {
  if (!gpsMap) return;
  if (gpsFinishLine) gpsMap.removeLayer(gpsFinishLine);
  if (gpsFinishEndpointLayer) gpsFinishEndpointLayer.clearLayers();
  gpsFinishMarkers = [];
  if (gpsFinishPoints.length !== 2) return;

  const latLngs = gpsFinishPoints.map(point => [point.lat, point.lon]);
  gpsFinishLine = L.polyline(latLngs, {
    color: '#eab308',
    weight: 5,
    opacity: 0.95,
    interactive: false
  }).addTo(gpsMap);
}

function drawGpsFinishFirstPoint() {
  if (!gpsMap || gpsFinishPoints.length !== 1) return;
  if (gpsFinishEndpointLayer) gpsFinishEndpointLayer.clearLayers();
  const first = gpsFinishPoints[0];
  L.marker([first.lat, first.lon], {
    interactive: false,
    icon: L.divIcon({ className: 'gps-finish-marker', html: '<div class="gps-finish-line-icon"></div>', iconSize: [18, 18], iconAnchor: [9, 9] })
  }).addTo(gpsFinishEndpointLayer);
  if (gpsFinishPreviewLine) gpsMap.removeLayer(gpsFinishPreviewLine);
  gpsFinishPreviewLine = L.polyline([[first.lat, first.lon], [first.lat, first.lon]], {
    color: '#eab308',
    weight: 4,
    opacity: 0.8,
    dashArray: '7 6',
    interactive: false
  }).addTo(gpsMap);
}

function updateGpsFinishPreview(event) {
  if (!gpsLapSelectionActive || gpsFinishPoints.length !== 1 || !gpsFinishPreviewLine) return;
  const first = gpsFinishPoints[0];
  gpsFinishPreviewLine.setLatLngs([[first.lat, first.lon], [event.latlng.lat, event.latlng.lng]]);
}

function findGpsLineCrossings(linePoints, minimumSpeed = 1) {
  if (!Array.isArray(linePoints) || linePoints.length !== 2) return [];
  const origin = {
    lat: (linePoints[0].lat + linePoints[1].lat) * 0.5,
    lon: (linePoints[0].lon + linePoints[1].lon) * 0.5
  };
  const a = latLonToLocalMeters(linePoints[0], origin);
  const b = latLonToLocalMeters(linePoints[1], origin);
  const line = { x: b.x - a.x, y: b.y - a.y };
  const lineLengthSq = line.x * line.x + line.y * line.y;
  if (lineLengthSq < 4) return [];
  const crossings = [];
  for (let index = 1; index < gpsLapPoints.length; index += 1) {
    const previous = gpsLapPoints[index - 1];
    const current = gpsLapPoints[index];
    const elapsed = current.time - previous.time;
    if (!(elapsed > 0 && elapsed <= 2)) continue;
    const p = latLonToLocalMeters(previous, origin);
    const q = latLonToLocalMeters(current, origin);
    const sidePrevious = cross2(line, { x: p.x - a.x, y: p.y - a.y });
    const sideCurrent = cross2(line, { x: q.x - a.x, y: q.y - a.y });
    if (sidePrevious === 0 || sideCurrent === 0 || sidePrevious * sideCurrent >= 0) continue;
    const fraction = sidePrevious / (sidePrevious - sideCurrent);
    const intersection = { x: p.x + (q.x - p.x) * fraction, y: p.y + (q.y - p.y) * fraction };
    const lineRatio = ((intersection.x - a.x) * line.x + (intersection.y - a.y) * line.y) / lineLengthSq;
    if (fraction < 0 || fraction > 1 || lineRatio < 0 || lineRatio > 1) continue;
    const speed = previous.speed + (current.speed - previous.speed) * fraction;
    if (speed < minimumSpeed) continue;
    crossings.push({
      time: previous.time + elapsed * fraction,
      speed,
      lat: previous.lat + (current.lat - previous.lat) * fraction,
      lon: previous.lon + (current.lon - previous.lon) * fraction
    });
  }
  return crossings;
}

function saveGpsFixedLines() {
  if (gpsFinishPoints.length !== 2) return;
  try {
    localStorage.setItem(GPS_FIXED_LINES_STORAGE_KEY, JSON.stringify({
      finish: gpsFinishPoints,
      checkpoints: gpsCheckpoints.slice(0, 2),
      savedAt: new Date().toISOString()
    }));
  } catch (error) { /* local storage may be unavailable */ }
}

function removeSavedGpsFixedLines() {
  try { localStorage.removeItem(GPS_FIXED_LINES_STORAGE_KEY); } catch (error) { /* ignore */ }
}

function verifyGpsFixedLinesPassword(actionLabel = '고정선을 변경') {
  return window.confirm(`${actionLabel}하시겠습니까?`);
}

function restoreGpsFixedLines() {
  if (gpsLapPoints.length < 2) return false;
  let saved;
  try { saved = JSON.parse(localStorage.getItem(GPS_FIXED_LINES_STORAGE_KEY) || 'null'); } catch (error) { return false; }
  if (!saved || !Array.isArray(saved.finish) || saved.finish.length !== 2) return false;
  const middle = {
    lat: (saved.finish[0].lat + saved.finish[1].lat) * 0.5,
    lon: (saved.finish[0].lon + saved.finish[1].lon) * 0.5
  };
  const nearest = gpsLapPoints.reduce((best, point) => Math.min(best, distanceMeters(middle, point)), Infinity);
  if (nearest > 120) {
    setGpsLapStatus('저장된 고정선은 다른 트랙 좌표라 적용하지 않았습니다.', 'warn');
    return false;
  }
  gpsFinishPoints = saved.finish.map(point => ({ lat: Number(point.lat), lon: Number(point.lon) }));
  gpsCheckpoints = Array.isArray(saved.checkpoints)
    ? saved.checkpoints.slice(0, 2).filter(line => Array.isArray(line) && line.length === 2)
    : [];
  drawGpsFinishLine();
  drawGpsCheckpoints();
  if (gpsLapClear) gpsLapClear.disabled = false;
  calculateGpsLaps();
  updateGpsVideoControlAvailability();
  setGpsLapStatus(`고정 피니시라인과 체크포인트 ${gpsCheckpoints.length}개를 불러왔습니다.`, 'ok');
  return true;
}

function drawGpsCheckpoints() {
  gpsCheckpointLayer?.clearLayers();
  gpsCheckpoints.forEach((checkpoint, index) => {
    const color = '#06b6d4';
    L.polyline(checkpoint.map(point => [point.lat, point.lon]), {
      color,
      weight: 4,
      opacity: 0.95,
      interactive: false
    }).addTo(gpsCheckpointLayer);
    const middle = {
      lat: (checkpoint[0].lat + checkpoint[1].lat) * 0.5,
      lon: (checkpoint[0].lon + checkpoint[1].lon) * 0.5
    };
    L.marker([middle.lat, middle.lon], {
      interactive: false,
      icon: L.divIcon({ className: '', html: `<div class="gps-checkpoint-label">CP${index + 1}</div>`, iconSize: [36, 18], iconAnchor: [18, 9] })
    }).addTo(gpsCheckpointLayer);
  });
  if (gpsCheckpointCount) gpsCheckpointCount.textContent = `${gpsCheckpoints.length} CP`;
  if (gpsCheckpointClear) gpsCheckpointClear.disabled = gpsFinishPoints.length !== 2 && !gpsCheckpoints.length;
  if (gpsCheckpointAdd) gpsCheckpointAdd.disabled = !gpsLapResults.length || gpsCheckpoints.length >= 2;
  if (gpsSectorToggle) gpsSectorToggle.disabled = !gpsCheckpoints.length;
}

function clearGpsCheckpoints(removeSaved = false) {
  gpsCheckpointSelectionActive = false;
  gpsCheckpointDraft = [];
  gpsCheckpoints = [];
  if (gpsCheckpointPreviewLine && gpsMap) gpsMap.removeLayer(gpsCheckpointPreviewLine);
  gpsCheckpointPreviewLine = null;
  gpsCheckpointLayer?.clearLayers();
  gpsCheckpointDraftLayer?.clearLayers();
  gpsCheckpointAdd?.classList.remove('active');
  if (gpsCheckpointAdd) gpsCheckpointAdd.disabled = true;
  if (gpsCheckpointClear) gpsCheckpointClear.disabled = true;
  if (gpsSectorToggle) gpsSectorToggle.disabled = true;
  if (gpsCheckpointCount) gpsCheckpointCount.textContent = '0 CP';
  if (gpsSectorCard) gpsSectorCard.hidden = true;
  if (gpsSectorTable) gpsSectorTable.innerHTML = '';
  if (gpsSectorOverlayTable) gpsSectorOverlayTable.innerHTML = '';
  if (gpsSectorOverlay) gpsSectorOverlay.hidden = true;
  if (removeSaved) removeSavedGpsFixedLines();
}

function beginGpsCheckpointSelection() {
  if (gpsFinishPoints.length !== 2 || !gpsLapResults.length) {
    setGpsLapStatus('먼저 피니시 라인을 설정해 랩을 계산하십시오.', 'warn');
    return;
  }
  if (gpsCheckpoints.length >= 2) {
    setGpsLapStatus('고정 체크포인트는 최대 2개입니다. 변경하려면 체크포인트 삭제 후 다시 지정하십시오.', 'warn');
    return;
  }
  setGpsPlayback(false);
  gpsLapSelectionActive = false;
  gpsCheckpointSelectionActive = true;
  gpsCheckpointDraft = [];
  gpsCheckpointDraftLayer?.clearLayers();
  gpsCheckpointAdd?.classList.add('active');
  gpsMap?.getContainer().classList.add('gps-lap-selecting');
  setGpsLapStatus(`CP${gpsCheckpoints.length + 1}의 첫 번째 끝점을 클릭하십시오.`, 'warn');
}

function cancelGpsCheckpointSelection() {
  if (!gpsCheckpointSelectionActive) return false;
  gpsCheckpointSelectionActive = false;
  gpsCheckpointDraft = [];
  if (gpsCheckpointPreviewLine && gpsMap) gpsMap.removeLayer(gpsCheckpointPreviewLine);
  gpsCheckpointPreviewLine = null;
  gpsCheckpointDraftLayer?.clearLayers();
  gpsCheckpointAdd?.classList.remove('active');
  gpsMap?.getContainer().classList.remove('gps-lap-selecting');
  setGpsLapStatus('체크포인트 설정을 취소했습니다.');
  return true;
}

function handleGpsCheckpointMapClick(event) {
  if (!gpsCheckpointSelectionActive) return;
  gpsCheckpointDraft.push({ lat: event.latlng.lat, lon: event.latlng.lng });
  if (gpsCheckpointDraft.length === 1) {
    const first = gpsCheckpointDraft[0];
    L.circleMarker([first.lat, first.lon], { radius: 5, color: '#fff', weight: 2, fillColor: '#06b6d4', fillOpacity: 1, interactive: false }).addTo(gpsCheckpointDraftLayer);
    gpsCheckpointPreviewLine = L.polyline([[first.lat, first.lon], [first.lat, first.lon]], { color: '#06b6d4', weight: 3, dashArray: '6 5', interactive: false }).addTo(gpsMap);
    setGpsLapStatus(`CP${gpsCheckpoints.length + 1}의 반대쪽 끝점을 클릭하십시오. Esc로 취소할 수 있습니다.`, 'warn');
    return;
  }
  gpsCheckpoints.push(gpsCheckpointDraft.slice(0, 2));
  gpsCheckpointSelectionActive = false;
  gpsCheckpointDraft = [];
  if (gpsCheckpointPreviewLine && gpsMap) gpsMap.removeLayer(gpsCheckpointPreviewLine);
  gpsCheckpointPreviewLine = null;
  gpsCheckpointDraftLayer?.clearLayers();
  gpsCheckpointAdd?.classList.remove('active');
  gpsMap?.getContainer().classList.remove('gps-lap-selecting');
  drawGpsCheckpoints();
  saveGpsFixedLines();
  renderGpsSectorComparison();
  setGpsLapStatus(gpsCheckpoints.length === 2 ? '고정 체크포인트 2개 저장 완료' : 'CP1 저장 완료 · CP2를 주행 순서대로 추가하십시오.', 'ok');
}

function updateGpsCheckpointPreview(event) {
  if (!gpsCheckpointSelectionActive || gpsCheckpointDraft.length !== 1 || !gpsCheckpointPreviewLine) return;
  const first = gpsCheckpointDraft[0];
  gpsCheckpointPreviewLine.setLatLngs([[first.lat, first.lon], [event.latlng.lat, event.latlng.lng]]);
}

function updateGpsCursorScale() {
  if (!gpsMap) return;
  const zoom = gpsMap.getZoom();
  const scale = Math.max(0.72, Math.min(1.18, 0.72 + (zoom - 7) * 0.031));
  const marker = gpsCursorMarker?.getElement()?.querySelector('.gps-position-cursor');
  if (marker) marker.style.setProperty('--gps-cursor-scale', scale.toFixed(3));
  gpsCompareMarkers.forEach(compareMarker => {
    compareMarker.getElement()?.querySelector('.gps-position-cursor')?.style.setProperty('--gps-cursor-scale', scale.toFixed(3));
  });
}

function updateGpsCursorLapColor(targetTime) {
  const marker = gpsCursorMarker?.getElement()?.querySelector('.gps-position-cursor');
  const selectedSingleLap = gpsSelectedLapIndices.length === 1 ? gpsSelectedLapIndices[0] : -1;
  const lapIndex = selectedSingleLap >= 0
    ? selectedSingleLap
    : gpsLapResults.findIndex(lap => targetTime >= lap.startTime && targetTime <= lap.endTime);
  const color = lapIndex >= 0 ? GPS_LAP_COLORS[lapIndex % GPS_LAP_COLORS.length] : '#00bfe8';
  if (marker) marker.style.setProperty('--gps-cursor-color', color);
  if (gpsFullscreenLapTimes) {
    gpsFullscreenLapTimes.querySelectorAll('[data-lap-time-row]').forEach(row => {
      row.classList.toggle('active', Number(row.dataset.lapTimeRow) === lapIndex);
    });
    const live = gpsFullscreenLapTimes.querySelector('[data-lap-live]');
    if (live) {
      if (lapIndex >= 0) {
        const lap = gpsLapResults[lapIndex];
        const elapsed = Math.max(0, Math.min(lap.duration, targetTime - lap.startTime));
        live.textContent = `LAP ${lap.number} · ${formatLapTime(elapsed)}`;
        live.style.color = color;
      } else {
        live.textContent = '완성된 랩 구간 밖';
        live.style.color = '';
      }
    }
  }
}

function renderFullscreenLapTimes(laps, best) {
  if (!gpsFullscreenLapTimes) return;
  const bestIndex = laps.findIndex(lap => Math.abs(lap.duration - best) < 0.0005);
  const bestLabel = `BEST ${formatLapTime(best)}`;
  gpsFullscreenLapTimes.hidden = !laps.length;
  gpsFullscreenLapTimes.innerHTML = laps.length ? `
    <div class="gps-fs-lap-head"><span>Lap Times</span><strong>${bestLabel}</strong></div>
    <div class="gps-fs-lap-live" data-lap-live>완성된 랩 구간 밖</div>
    <button type="button" class="gps-fs-lap-all${gpsSelectedLapIndices.length === 0 ? ' selected' : ''}" data-lap-panel-view="all">전체 랩 보기</button>
    <div class="gps-fs-lap-list">${laps.map((lap, index) => `
      <button type="button" class="${index === bestIndex ? 'best ' : ''}${gpsSelectedLapIndices.includes(index) ? 'selected' : ''}" data-lap-time-row="${index}" data-lap-panel-view="${index}" style="--lap-color:${GPS_LAP_COLORS[index % GPS_LAP_COLORS.length]}">
        <span><i></i>LAP ${lap.number}${index === bestIndex ? '<b class="gps-best-star">★</b>' : ''}</span><strong>${formatLapTime(lap.duration)}<small>${formatGpsLapDistance(lap.distanceMeters)}</small></strong>
      </button>`).join('')}</div>` : '';
  updateGoProComparisonLayout();
}

function refreshGpsFullscreenOverlays() {
  if (gpsLapResults.length) {
    const best = Math.min(...gpsLapResults.map(lap => lap.duration));
    renderFullscreenLapTimes(gpsLapResults, best);
  } else if (gpsFullscreenLapTimes) {
    gpsFullscreenLapTimes.hidden = true;
  }
  updateGpsCursorLapColor(Number(scrollBar?.value));
  const targetTime = Number(scrollBar?.value) || 0;
  const rowIndex = globalData.length ? findGlobalIndexAtTime(targetTime) : -1;
  const row = rowIndex >= 0 ? globalData[rowIndex] : null;
  if (row && gpsFullscreenSpeedValue) {
    gpsFullscreenSpeedValue.textContent = (Number(row.gps_speed_kmh) || 0).toFixed(1);
  }
  if (gpsFullscreenPlayTime) {
    gpsFullscreenPlayTime.textContent = `${formatGpsClock(gpsClockAtTelemetryTime(targetTime, rowIndex))} KST`;
  }
}

function drawGpsLapRoutes(laps) {
  if (gpsLapRouteLayer) gpsLapRouteLayer.clearLayers();
  gpsLapRouteLines = [];
  gpsSelectedLapIndex = -1;
  gpsSelectedLapIndices = [];
  if (gpsRouteLine) gpsRouteLine.setStyle({ opacity: laps.length ? 0.22 : 0.8, weight: laps.length ? 3 : 5 });
  if (gpsLapMapLegend) {
    gpsLapMapLegend.hidden = !laps.length;
    gpsLapMapLegend.innerHTML = laps.length
      ? `<button type="button" class="active" data-lap-view="all">전체</button>` + laps.map((lap, index) =>
          `<button type="button" data-lap-view="${index}"><i style="--lap-color:${GPS_LAP_COLORS[index % GPS_LAP_COLORS.length]}"></i>LAP ${lap.number}</button>`
        ).join('')
      : '';
  }
  if (!gpsLapRouteLayer || !laps.length) return;

  laps.forEach((lap, index) => {
    const coords = [[lap.startLat, lap.startLon]];
    gpsLapPoints.forEach(point => {
      if (point.time > lap.startTime && point.time < lap.endTime) coords.push([point.lat, point.lon]);
    });
    coords.push([lap.endLat, lap.endLon]);
    if (coords.length < 2) return;
    const line = L.polyline(coords, {
      color: GPS_LAP_COLORS[index % GPS_LAP_COLORS.length],
      weight: 6,
      opacity: 0.92,
      interactive: false
    });
    gpsLapRouteLines[index] = line;
    line.addTo(gpsLapRouteLayer);
  });
  if (gpsCursorMarker) gpsCursorMarker.setZIndexOffset(10000);
}

function syncGpsTimelineRange(minTime, maxTime, value) {
  const safeValue = Math.max(minTime, Math.min(maxTime, Number(value) || minTime));
  scrollBar.min = minTime.toFixed(2);
  scrollBar.max = maxTime.toFixed(2);
  scrollBar.step = '0.04';
  scrollBar.value = safeValue.toFixed(2);
  if (gpsFullscreenTimeline) {
    gpsFullscreenTimeline.min = scrollBar.min;
    gpsFullscreenTimeline.max = scrollBar.max;
    gpsFullscreenTimeline.step = scrollBar.step;
    gpsFullscreenTimeline.value = scrollBar.value;
    updateGpsFullscreenTimelineVisual();
  }
  updateGpsDetailChartRange(minTime, maxTime);
  return safeValue;
}

function updateGpsFullscreenTimelineVisual() {
  if (!gpsFullscreenTimeline) return;
  const min = Number(gpsFullscreenTimeline.min) || 0;
  const max = Number(gpsFullscreenTimeline.max) || 0;
  const value = Number(gpsFullscreenTimeline.value) || min;
  const ratio = max > min ? Math.max(0, Math.min(1, (value - min) / (max - min))) : 0;
  const width = gpsFullscreenTimeline.clientWidth;
  const progress = width > 30
    ? `${(15 + ratio * (width - 30)).toFixed(2)}px`
    : `${(ratio * 100).toFixed(4)}%`;
  gpsFullscreenTimeline.style.setProperty('--timeline-progress', progress);
}

const gpsDetailCursorPlugin = {
  id: 'gpsDetailCursor',
  afterDatasetsDraw(chart) {
    const time = chart.$gpsCursorTime;
    const xScale = chart.scales?.x;
    const area = chart.chartArea;
    if (!Number.isFinite(time) || !xScale || !area || time < xScale.min || time > xScale.max) return;
    const x = xScale.getPixelForValue(time);
    const ctx = chart.ctx;
    ctx.save();
    ctx.strokeStyle = '#ff7a1a';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, area.top);
    ctx.lineTo(x, area.bottom);
    ctx.stroke();
    chart.data.datasets.forEach((dataset, datasetIndex) => {
      const data = dataset.data || [];
      if (!data.length) return;
      let lo = 0, hi = data.length - 1;
      while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (Number(data[mid].x) < time) lo = mid + 1;
        else hi = mid;
      }
      const left = Math.max(0, lo - 1);
      const nearest = Math.abs(Number(data[left].x) - time) <= Math.abs(Number(data[lo].x) - time) ? data[left] : data[lo];
      const value = Number(nearest?.y);
      const yScale = chart.scales[dataset.yAxisID || 'y'];
      if (!Number.isFinite(value) || !yScale) return;
      const y = yScale.getPixelForValue(value);
      if (y < area.top || y > area.bottom) return;
      ctx.beginPath();
      ctx.arc(x, y, 3.5, 0, Math.PI * 2);
      ctx.fillStyle = dataset.borderColor || '#2563eb';
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    });
    ctx.restore();
  }
};

function destroyGpsDetailCharts() {
  gpsDetailCharts.forEach(chart => chart?.destroy());
  gpsDetailCharts = [];
  gpsDetailSourceData = null;
}

function cloneGpsDetailDatasets(sourceChart, palette) {
  return (sourceChart?.data?.datasets || []).map((dataset, index) => ({
    label: dataset.label || '',
    data: dataset.data,
    borderColor: palette[index] || dataset.borderColor,
    backgroundColor: dataset.backgroundColor,
    borderWidth: Math.max(1.2, Number(dataset.borderWidth) || 1.2),
    pointRadius: 0,
    stepped: dataset.stepped,
    tension: dataset.tension || 0,
    fill: false,
    yAxisID: dataset.yAxisID
  }));
}

function ensureGpsDetailCharts() {
  if (gpsDetailCharts.length && gpsDetailSourceData === globalData) return;
  destroyGpsDetailCharts();
  const specs = [
    ['gps-detail-speed', chartSpeed, ['#f97316', '#2563eb', '#16a34a']],
    ['gps-detail-rpm', chartRpm, ['#dc2626']],
    ['gps-detail-gear', chartGear, ['#2563eb']],
    ['gps-detail-steering', chartSteering, ['#db2777']],
    ['gps-detail-throttle-brake', chartThrottleBrake, ['#16a34a', '#dc2626']]
  ];
  gpsDetailCharts = specs.map(([id, source, palette]) => {
    const canvas = document.getElementById(id);
    if (!canvas || !source) return null;
    const min = Number(scrollBar.min) || currentStartSec;
    const max = Number(scrollBar.max) || currentEndSec;
    const sourceYOptions = source.options?.scales?.y || {};
    const sourceYScale = source.scales?.y;
    return new Chart(canvas.getContext('2d'), {
      type: 'line',
      data: { datasets: cloneGpsDetailDatasets(source, palette) },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        parsing: false,
        normalized: true,
        interaction: { enabled: false },
        plugins: { legend: { display: false }, tooltip: { enabled: false } },
        scales: {
          x: { type: 'linear', min, max, display: false, grid: { display: false } },
          y: {
            min: sourceYOptions.min ?? sourceYScale?.min,
            max: sourceYOptions.max ?? sourceYScale?.max,
            afterFit(scale) { scale.width = 48; },
            grid: { color: 'rgba(71, 85, 105, 0.14)' },
            ticks: {
              color: '#475569',
              font: { size: 8 },
              maxTicksLimit: 4,
              stepSize: sourceYOptions.ticks?.stepSize,
              callback: sourceYOptions.ticks?.callback
            }
          }
        }
      }
    });
  }).filter(Boolean);
  gpsDetailCharts.forEach(ensureGpsDetailCursorOverlay);
  gpsDetailSourceData = globalData;
}

function updateGpsDetailChartRange(minTime, maxTime) {
  gpsDetailCharts.forEach(chart => {
    chart.options.scales.x.min = minTime;
    chart.options.scales.x.max = maxTime;
    chart.update('none');
  });
}

function updateGpsDetailCursors(targetTime) {
  if (!gpsFullscreenDetail?.classList.contains('open')) return;
  updateGpsDetailReadouts(targetTime);
  gpsDetailCharts.forEach(chart => {
    chart.$gpsCursorTime = targetTime;
    updateGpsDetailCursorOverlay(chart, targetTime);
  });
}

function ensureGpsDetailCursorOverlay(chart) {
  const section = chart.canvas.closest('section');
  if (!section || section.querySelector('.gps-detail-cursor-overlay')) return;
  const overlay = document.createElement('div');
  overlay.className = 'gps-detail-cursor-overlay';
  overlay.innerHTML = '<i class="gps-detail-cursor-line"></i><div class="gps-detail-cursor-dots"></div>';
  section.appendChild(overlay);
}

function updateGpsDetailCursorOverlay(chart, targetTime) {
  const section = chart.canvas.closest('section');
  const overlay = section?.querySelector('.gps-detail-cursor-overlay');
  const xScale = chart.scales?.x;
  const area = chart.chartArea;
  if (!overlay || !xScale || !area || targetTime < xScale.min || targetTime > xScale.max) {
    if (overlay) overlay.style.display = 'none';
    return;
  }
  overlay.style.display = 'block';
  const canvasLeft = chart.canvas.offsetLeft;
  const canvasTop = chart.canvas.offsetTop;
  const x = canvasLeft + xScale.getPixelForValue(targetTime);
  const line = overlay.querySelector('.gps-detail-cursor-line');
  line.style.left = `${x}px`;
  line.style.top = `${canvasTop + area.top}px`;
  line.style.height = `${area.bottom - area.top}px`;

  const dots = overlay.querySelector('.gps-detail-cursor-dots');
  const datasets = chart.data.datasets;
  while (dots.children.length < datasets.length) dots.appendChild(document.createElement('i'));
  while (dots.children.length > datasets.length) dots.lastElementChild.remove();
  datasets.forEach((dataset, datasetIndex) => {
    const dot = dots.children[datasetIndex];
    const lineElement = chart.getDatasetMeta(datasetIndex)?.dataset;
    const interpolated = lineElement?.interpolate({ x: xScale.getPixelForValue(targetTime) }, 'x');
    const point = Array.isArray(interpolated) ? interpolated[0] : interpolated;
    if (!Number.isFinite(point?.x) || !Number.isFinite(point?.y)) {
      dot.style.display = 'none';
      return;
    }
    const y = canvasTop + point.y;
    if (y < canvasTop + area.top || y > canvasTop + area.bottom) {
      dot.style.display = 'none';
      return;
    }
    const dotX = canvasLeft + point.x;
    dot.style.display = 'block';
    dot.style.transform = `translate3d(${dotX - 4}px, ${y - 4}px, 0)`;
    dot.style.background = dataset.borderColor || '#2563eb';
  });
}

function detailChannelValue(key, row, index, fallback) {
  if (typeof channelValueAt === 'function' && index >= 0) {
    const value = channelValueAt(key, index);
    if (Number.isFinite(value)) return value;
  }
  return fallback(row);
}

function updateGpsDetailReadouts(targetTime) {
  const primaryLap = gpsSelectedLapIndices.length > 1 ? gpsLapResults[gpsSelectedLapIndices[0]] : null;
  const lookupTime = primaryLap ? primaryLap.startTime + Math.min(targetTime, primaryLap.duration) : targetTime;
  const index = globalData.length ? findGlobalIndexAtTime(lookupTime) : -1;
  const row = index >= 0 ? globalData[index] : null;
  if (!row) return;
  const fl = detailChannelValue('fl_speed', row, index, r => Number(r.fl_speed_kmh) || 0);
  const rl = detailChannelValue('rl_speed', row, index, r => Number(r.rl_speed_kmh) || 0);
  const rr = detailChannelValue('rr_speed', row, index, r => Number(r.rr_speed_kmh) || 0);
  const rpm = detailChannelValue('rpm', row, index, r => Number(r.rpm) || 0);
  const gear = detailChannelValue('gear', row, index, r => Number(r.gear) || 0);
  const steering = detailChannelValue('steering', row, index, r => getCalibratedSteering(r.steering_raw));
  const throttle = detailChannelValue('throttle', row, index, r => Number(r.decoded_tps) || 0);
  const brake = detailChannelValue('brake', row, index, r => getCalibratedBrake(r.front_brake_raw));
  if (gpsDetailSpeedValue) gpsDetailSpeedValue.textContent = `FL ${fl.toFixed(1)} · RL ${rl.toFixed(1)} · RR ${rr.toFixed(1)} km/h`;
  if (gpsDetailRpmValue) gpsDetailRpmValue.textContent = `${Math.round(rpm)} rpm`;
  if (gpsDetailGearValue) gpsDetailGearValue.textContent = gear > 0 ? String(Math.round(gear)) : 'N';
  if (gpsDetailSteeringValue) gpsDetailSteeringValue.textContent = `${steering >= 0 ? '+' : ''}${steering.toFixed(1)}°`;
  if (gpsDetailPedalValue) gpsDetailPedalValue.textContent = `T ${throttle.toFixed(1)} · B ${brake.toFixed(1)} %`;
}

function refitGpsMapToCurrentLapView() {
  if (!gpsMap) return;
  const compactVideoMap = gpsMap.getContainer()?.closest('.gps-map-stage')?.classList.contains('gps-video-loaded');
  const padding = compactVideoMap ? [4, 4] : [35, 35];
  if (gpsSelectedLapIndices.length) {
    const lines = gpsSelectedLapIndices.map(index => gpsLapRouteLines[index]).filter(Boolean);
    const bounds = lines.length ? L.featureGroup(lines).getBounds() : null;
    if (bounds?.isValid()) gpsMap.fitBounds(bounds, { padding, maxZoom: 20 });
  } else {
    const lines = gpsLapRouteLines.filter(Boolean);
    const bounds = lines.length ? L.featureGroup(lines).getBounds() : gpsRouteLine?.getBounds();
    if (bounds?.isValid()) gpsMap.fitBounds(bounds, { padding: compactVideoMap ? [4, 4] : [30, 30], maxZoom: 20 });
  }
}

function clearGpsCompareMarkers() {
  gpsCompareMarkers.forEach(marker => gpsMap?.removeLayer(marker));
  gpsCompareMarkers = [];
  if (gpsCursorMarker) gpsCursorMarker.setOpacity(1);
}

function getGpsLapPositionAtTime(lap, targetTime) {
  if (targetTime <= lap.startTime) return { lat: lap.startLat, lon: lap.startLon };
  if (targetTime >= lap.endTime) return { lat: lap.endLat, lon: lap.endLon };

  let low = 0;
  let high = gpsLapPoints.length;
  while (low < high) {
    const middle = (low + high) >> 1;
    if (gpsLapPoints[middle].time < targetTime) low = middle + 1;
    else high = middle;
  }

  const previous = low > 0 && gpsLapPoints[low - 1].time > lap.startTime
    ? gpsLapPoints[low - 1]
    : { lat: lap.startLat, lon: lap.startLon, time: lap.startTime };
  const next = low < gpsLapPoints.length && gpsLapPoints[low].time < lap.endTime
    ? gpsLapPoints[low]
    : { lat: lap.endLat, lon: lap.endLon, time: lap.endTime };
  const elapsed = next.time - previous.time;
  if (!(elapsed > 0) || elapsed > 10) return { lat: previous.lat, lon: previous.lon };
  const ratio = Math.max(0, Math.min(1, (targetTime - previous.time) / elapsed));
  return {
    lat: previous.lat + (next.lat - previous.lat) * ratio,
    lon: previous.lon + (next.lon - previous.lon) * ratio
  };
}

function updateGpsCompareMarkers(relativeTime) {
  if (gpsSelectedLapIndices.length < 2 || !gpsMap) return;
  if (gpsCursorMarker) gpsCursorMarker.setOpacity(0);
  while (gpsCompareMarkers.length > gpsSelectedLapIndices.length) {
    gpsMap.removeLayer(gpsCompareMarkers.pop());
  }
  gpsSelectedLapIndices.forEach((lapIndex, markerIndex) => {
    const lap = gpsLapResults[lapIndex];
    const absoluteTime = lap.startTime + Math.min(relativeTime, lap.duration);
    const position = getGpsLapPositionAtTime(lap, absoluteTime);
    if (!position) return;
    let marker = gpsCompareMarkers[markerIndex];
    if (!marker) {
      const color = GPS_LAP_COLORS[lapIndex % GPS_LAP_COLORS.length];
      const icon = L.divIcon({
        className: 'custom-div-icon',
        html: `<div class="gps-position-cursor gps-compare-cursor" style="--gps-cursor-color:${color}"><b>${lap.number}</b></div>`,
        iconSize: [16, 16], iconAnchor: [8, 8]
      });
      marker = L.marker([position.lat, position.lon], { icon, zIndexOffset: 11000 + markerIndex }).addTo(gpsMap);
      gpsCompareMarkers[markerIndex] = marker;
    } else {
      marker.setLatLng([position.lat, position.lon]);
    }
  });
  updateGpsCursorScale();
}

function rebuildGpsDetailChartsForSelection() {
  if (!gpsFullscreenDetail?.classList.contains('open')) return;
  destroyGpsDetailCharts();
  ensureGpsDetailCharts();
  if (gpsSelectedLapIndices.length < 2) return;
  const sources = [chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake];
  const sourceDatasetIndexes = [[0], [0], [0], [0], [0, 1]];
  gpsDetailCharts.forEach((chart, chartIndex) => {
    const source = sources[chartIndex];
    const selectedDatasets = [];
    gpsSelectedLapIndices.forEach(lapIndex => {
      const lap = gpsLapResults[lapIndex];
      const color = GPS_LAP_COLORS[lapIndex % GPS_LAP_COLORS.length];
      sourceDatasetIndexes[chartIndex].forEach((datasetIndex, subIndex) => {
        const sourceDataset = source.data.datasets[datasetIndex];
        const data = (sourceDataset.data || [])
          .filter(point => point.x >= lap.startTime && point.x <= lap.endTime)
          .map(point => ({ x: point.x - lap.startTime, y: point.y }));
        selectedDatasets.push({
          label: `LAP ${lap.number}${subIndex ? ' Brake' : ''}`,
          data,
          borderColor: color,
          borderWidth: subIndex ? 1.2 : 1.8,
          borderDash: subIndex ? [5, 3] : [],
          pointRadius: 0,
          stepped: sourceDataset.stepped,
          fill: false
        });
      });
    });
    chart.data.datasets = selectedDatasets;
    chart.options.scales.x.min = 0;
    chart.options.scales.x.max = Math.max(...gpsSelectedLapIndices.map(index => gpsLapResults[index].duration));
    chart.update('none');
  });
}

function closeGpsFullscreenDetail() {
  const stage = gpsFullscreenDetailToggle?.closest('.gps-map-stage');
  gpsFullscreenDetail?.classList.remove('open');
  stage?.classList.remove('gps-detail-open');
  if (gpsFullscreenDetailToggle) gpsFullscreenDetailToggle.textContent = '상세정보 ›';
}

function selectGpsLapView(index) {
  if (!gpsLapResults.length || !gpsLapRouteLayer) return;
  const validLap = Number.isInteger(index) && index >= 0 && index < gpsLapResults.length;
  const wasSelected = validLap && gpsSelectedLapIndices.includes(index);
  if (!validLap) gpsSelectedLapIndices = [];
  else if (wasSelected) gpsSelectedLapIndices = gpsSelectedLapIndices.filter(value => value !== index);
  else gpsSelectedLapIndices = [...gpsSelectedLapIndices, index].sort((a, b) => a - b);
  if (validLap && !wasSelected && gpsSelectedLapIndices.length > 1) gpsGoProCompareLapIndex = index;
  if (!gpsSelectedLapIndices.includes(gpsGoProCompareLapIndex)) gpsGoProCompareLapIndex = -1;
  gpsSelectedLapIndex = gpsSelectedLapIndices.length === 1 ? gpsSelectedLapIndices[0] : -1;
  const hasSelection = gpsSelectedLapIndices.length > 0;
  const isComparison = gpsSelectedLapIndices.length > 1;
  gpsLapRouteLayer.clearLayers();
  gpsLapRouteLines.forEach((line, lineIndex) => {
    if (!hasSelection || gpsSelectedLapIndices.includes(lineIndex)) line.addTo(gpsLapRouteLayer);
  });
  gpsLapMapLegend?.querySelectorAll('[data-lap-view]').forEach(button => {
    button.classList.toggle('active', button.dataset.lapView === 'all' ? !hasSelection : gpsSelectedLapIndices.includes(Number(button.dataset.lapView)));
  });
  gpsFullscreenLapTimes?.querySelectorAll('[data-lap-panel-view]').forEach(button => {
    button.classList.toggle('selected', button.dataset.lapPanelView === 'all' ? !hasSelection : gpsSelectedLapIndices.includes(Number(button.dataset.lapPanelView)));
  });
  updateGoProComparisonLayout();

  setGpsPlayback(false);
  clearGpsCompareMarkers();
  rebuildGpsDetailChartsForSelection();
  if (gpsSelectedLapIndices.length === 1) {
    const selectedIndex = gpsSelectedLapIndices[0];
    const lap = gpsLapResults[selectedIndex];
    const targetTime = syncGpsTimelineRange(lap.startTime, lap.endTime, lap.startTime);
    updateGpsCursorAtTime(targetTime);
    const line = gpsLapRouteLines[selectedIndex];
    if (line?.getBounds().isValid()) gpsMap.fitBounds(line.getBounds(), { padding: [45, 45], maxZoom: 20 });
  } else if (isComparison) {
    const maxDuration = Math.max(...gpsSelectedLapIndices.map(lapIndex => gpsLapResults[lapIndex].duration));
    const targetTime = syncGpsTimelineRange(0, maxDuration, 0);
    updateGpsCursorAtTime(targetTime);
    refitGpsMapToCurrentLapView();
  } else {
    const targetTime = syncGpsTimelineRange(currentStartSec, currentEndSec, scrollBar.value);
    updateGpsCursorAtTime(targetTime);
    const visibleLines = gpsLapRouteLines.filter(Boolean);
    if (visibleLines.length) {
      const bounds = L.featureGroup(visibleLines).getBounds();
      if (bounds.isValid()) gpsMap.fitBounds(bounds, { padding: [30, 30] });
    }
  }
  if (gpsCursorMarker) gpsCursorMarker.setZIndexOffset(10000);
  if (gpsCheckpoints.length) renderGpsSectorComparison();
}

function renderGpsLapResults(crossings, laps) {
  gpsLapResults = laps;
  if (gpsCheckpointAdd) gpsCheckpointAdd.disabled = !laps.length || gpsCheckpoints.length >= 2;
  gpsSelectedLapIndex = -1;
  gpsSelectedLapIndices = [];
  gpsGoProCompareLapIndex = -1;
  clearGpsCompareMarkers();
  if (gpsLapCount) gpsLapCount.textContent = `${laps.length} LAPS`;
  const best = laps.length ? Math.min(...laps.map(lap => lap.duration)) : NaN;
  const lapDistances = laps.map(lap => lap.distanceMeters).filter(Number.isFinite);
  const averageDistance = lapDistances.length
    ? lapDistances.reduce((sum, distance) => sum + distance, 0) / lapDistances.length
    : NaN;
  renderFullscreenLapTimes(laps, best);
  if (gpsLapBestTime) gpsLapBestTime.textContent = formatLapTime(best);
  if (gpsLapAverageDistance) gpsLapAverageDistance.textContent = formatGpsLapDistance(averageDistance);
  if (gpsLapFixSummary) {
    const timeBasis = laps.length && laps.every(lap => lap.timeBasis === 'gps') ? 'GPS 시각 기준' : '로거 경과시간 기준';
    gpsLapFixSummary.textContent = `${gpsLapPoints.length.toLocaleString()} GPS fixes · ${crossings.length}회 통과 · ${timeBasis}`;
  }

  if (gpsLapCrossingLayer) {
    gpsLapCrossingLayer.clearLayers();
    crossings.forEach((crossing, index) => {
      L.circleMarker([crossing.lat, crossing.lon], {
        radius: 4,
        color: '#fff',
        weight: 1,
        fillColor: index === 0 ? '#eab308' : '#10b981',
        fillOpacity: 1,
        interactive: false
      }).addTo(gpsLapCrossingLayer);
    });
  }
  drawGpsLapRoutes(laps);
  updateGpsCursorLapColor(Number(scrollBar?.value));
  if (tabGps?.classList.contains('active') && activeSampledData.length) {
    syncGpsTimelineRange(currentStartSec, currentEndSec, scrollBar.value);
  }

  if (!gpsLapList) return;
  if (!laps.length) {
    gpsLapList.innerHTML = '<div class="gps-lap-empty">라인 통과가 2회 이상 검출되지 않았습니다. 라인이 트랙 전체 폭을 가로지르는지 확인하십시오.</div>';
    return;
  }

  gpsLapList.innerHTML = laps.map(lap => {
    const isBest = Math.abs(lap.duration - best) < 0.0005;
    const delta = lap.duration - best;
    return `<details class="gps-lap-row${isBest ? ' best' : ''}">
      <summary>
        <span><i class="gps-lap-color-dot" style="--lap-color:${GPS_LAP_COLORS[(lap.number - 1) % GPS_LAP_COLORS.length]}"></i>LAP ${lap.number}</span>
        <span class="lap-time">${formatLapTime(lap.duration)}<small>${formatGpsLapDistance(lap.distanceMeters)}</small></span>
        <span class="lap-delta">${isBest ? 'BEST' : `+${delta.toFixed(3)}`}</span>
      </summary>
      <div class="gps-lap-detail">
        <span>CSV 구간 <strong>${lap.startTime.toFixed(2)}s → ${lap.endTime.toFixed(2)}s</strong></span>
        <span>GPS 시각 <strong>${formatGpsClock(lap.startGpsTime)} → ${formatGpsClock(lap.endGpsTime)}</strong></span>
        <span>GPS 주행거리 <strong>${formatGpsLapDistance(lap.distanceMeters)}</strong></span>
        <div class="gps-lap-jump-buttons">
          <button type="button" data-lap-time="${lap.startTime}">시작 지점으로 이동</button>
          <button type="button" data-lap-time="${lap.endTime}">종료 지점으로 이동</button>
        </div>
      </div>
    </details>`;
  }).join('');
  if (gpsCheckpointAdd) gpsCheckpointAdd.disabled = !laps.length || gpsCheckpoints.length >= 2;
  if (gpsCheckpoints.length) renderGpsSectorComparison();
}

function getGpsSectorMetrics(startTime, endTime, exitSpeed) {
  const startIndex = Math.max(0, findGlobalIndexAtTime(startTime));
  const endIndex = Math.max(startIndex, findGlobalIndexAtTime(endTime));
  let minimumSpeed = Infinity;
  let minimumIndex = startIndex;
  let brakeTime = NaN;
  for (let index = startIndex; index <= endIndex; index += 1) {
    const row = globalData[index];
    const speed = Number(row?.gps_speed_kmh) || 0;
    if (speed < minimumSpeed) {
      minimumSpeed = speed;
      minimumIndex = index;
    }
    if (!Number.isFinite(brakeTime) && getCalibratedBrake(row?.front_brake_raw) >= 5) brakeTime = row.time_sec;
  }
  let throttleTime = NaN;
  for (let index = minimumIndex; index <= endIndex; index += 1) {
    const row = globalData[index];
    if ((Number(row?.decoded_tps) || 0) >= 20) {
      throttleTime = row.time_sec;
      break;
    }
  }
  return {
    duration: endTime - startTime,
    exitSpeed: Number(exitSpeed) || 0,
    minimumSpeed: Number.isFinite(minimumSpeed) ? minimumSpeed : 0,
    brakeOffset: Number.isFinite(brakeTime) ? brakeTime - startTime : NaN,
    throttleOffset: Number.isFinite(throttleTime) ? throttleTime - startTime : NaN
  };
}

function renderGpsSectorComparison() {
  if (!gpsSectorCard || !gpsSectorTable) return;
  if (!gpsCheckpoints.length || !gpsLapResults.length) {
    gpsSectorCard.hidden = true;
    gpsSectorTable.innerHTML = '';
    return;
  }
  const checkpointCrossings = gpsCheckpoints.map(checkpoint => findGpsLineCrossings(checkpoint, 3));
  const lapIndexes = gpsSelectedLapIndices.length
    ? gpsSelectedLapIndices
    : gpsLapResults.map((_, index) => index);
  const sectors = gpsCheckpoints.map((_, checkpointIndex) => ({ name: `S${checkpointIndex + 1} → CP${checkpointIndex + 1}`, rows: [] }));
  sectors.push({ name: `S${gpsCheckpoints.length + 1} → FINISH`, rows: [] });

  lapIndexes.forEach(lapIndex => {
    const lap = gpsLapResults[lapIndex];
    let sectorStart = lap.startTime;
    let valid = true;
    checkpointCrossings.forEach((crossings, checkpointIndex) => {
      if (!valid) return;
      const crossing = crossings.find(item => item.time > sectorStart + 0.05 && item.time < lap.endTime - 0.05);
      if (!crossing) {
        sectors[checkpointIndex].rows.push({ lapIndex, missing: true });
        valid = false;
        return;
      }
      sectors[checkpointIndex].rows.push({ lapIndex, ...getGpsSectorMetrics(sectorStart, crossing.time, crossing.speed) });
      sectorStart = crossing.time;
    });
    if (valid) {
      sectors[sectors.length - 1].rows.push({
        lapIndex,
        ...getGpsSectorMetrics(sectorStart, lap.endTime, gpsSpeedAtTelemetryTime(lap.endTime))
      });
    } else {
      sectors[sectors.length - 1].rows.push({ lapIndex, missing: true });
    }
  });

  const comparisonHtml = sectors.map(sector => {
    const validRows = sector.rows.filter(row => !row.missing);
    const bestDuration = validRows.length ? Math.min(...validRows.map(row => row.duration)) : NaN;
    const rows = sector.rows.map(row => {
      const lap = gpsLapResults[row.lapIndex];
      if (row.missing) return `<tr><td style="--lap-color:${GPS_LAP_COLORS[row.lapIndex % GPS_LAP_COLORS.length]}"><i class="gps-lap-color-dot"></i>LAP ${lap.number}</td><td colspan="5">통과 기록 없음</td></tr>`;
      const bestClass = Math.abs(row.duration - bestDuration) < 0.0005 ? ' class="best-sector"' : '';
      return `<tr${bestClass}>
        <td style="--lap-color:${GPS_LAP_COLORS[row.lapIndex % GPS_LAP_COLORS.length]}"><i class="gps-lap-color-dot"></i>LAP ${lap.number}</td>
        <td>${row.duration.toFixed(3)} s</td>
        <td>${row.exitSpeed.toFixed(1)} km/h</td>
        <td>${row.minimumSpeed.toFixed(1)} km/h</td>
        <td>${Number.isFinite(row.brakeOffset) ? `+${row.brakeOffset.toFixed(2)} s` : '—'}</td>
        <td>${Number.isFinite(row.throttleOffset) ? `+${row.throttleOffset.toFixed(2)} s` : '—'}</td>
      </tr>`;
    }).join('');
    return `<section class="gps-sector-group"><h4>${sector.name}</h4><table><thead><tr><th>LAP</th><th>구간 기록</th><th>통과 속도</th><th>최저속도</th><th>브레이크 ≥5%</th><th>가속 ≥20%</th></tr></thead><tbody>${rows}</tbody></table></section>`;
  }).join('');
  gpsSectorTable.innerHTML = comparisonHtml;
  if (gpsSectorOverlayTable) gpsSectorOverlayTable.innerHTML = comparisonHtml;
  gpsSectorCard.hidden = false;
}

function closeGpsSectorOverlay() {
  if (gpsSectorOverlay) gpsSectorOverlay.hidden = true;
  gpsSectorToggle?.classList.remove('active');
}

function toggleGpsSectorOverlay() {
  if (!gpsCheckpoints.length) return;
  const shouldOpen = Boolean(gpsSectorOverlay?.hidden);
  if (shouldOpen) {
    closeGpsFullscreenDetail();
    renderGpsSectorComparison();
  }
  if (gpsSectorOverlay) gpsSectorOverlay.hidden = !shouldOpen;
  gpsSectorToggle?.classList.toggle('active', shouldOpen);
}

function updateGpsVideoControlAvailability() {
  const enabled = gpsFinishPoints.length === 2;
  if (gpsGoProFile) gpsGoProFile.disabled = !enabled;
  gpsGoProOpen?.classList.toggle('disabled', !enabled);
  gpsGoProOpen?.setAttribute('aria-disabled', String(!enabled));
  gpsYouTubeOpen?.setAttribute('aria-disabled', String(!enabled));
}

function requireGpsFinishLineForVideo(event) {
  if (gpsFinishPoints.length === 2) return true;
  event?.preventDefault();
  setGpsLapStatus('영상 동기화를 사용하려면 먼저 피니시 라인을 설정하십시오.', 'warn');
  return false;
}

function crossingElapsedSeconds(previous, current) {
  if (Number.isFinite(previous.gpsTime) && Number.isFinite(current.gpsTime)) {
    const gpsElapsed = current.gpsTime - previous.gpsTime;
    if (gpsElapsed > 0 && gpsElapsed < 3600) return { duration: gpsElapsed, basis: 'gps' };
  }
  return { duration: current.time - previous.time, basis: 'logger' };
}

function calculateGpsLaps() {
  if (gpsFinishPoints.length !== 2) return;
  if (gpsLapPoints.length < 2) {
    renderGpsLapResults([], []);
    setGpsLapStatus('유효한 GPS fix가 부족합니다.', 'warn');
    return;
  }

  const origin = {
    lat: (gpsFinishPoints[0].lat + gpsFinishPoints[1].lat) * 0.5,
    lon: (gpsFinishPoints[0].lon + gpsFinishPoints[1].lon) * 0.5
  };
  const a = latLonToLocalMeters(gpsFinishPoints[0], origin);
  const b = latLonToLocalMeters(gpsFinishPoints[1], origin);
  const line = { x: b.x - a.x, y: b.y - a.y };
  const lineLengthSq = line.x * line.x + line.y * line.y;
  if (lineLengthSq < 4) {
    setGpsLapStatus('피니시 라인은 2m 이상으로 지정하십시오.', 'warn');
    return;
  }

  const candidates = [];
  for (let i = 1; i < gpsLapPoints.length; i++) {
    const prev = gpsLapPoints[i - 1];
    const curr = gpsLapPoints[i];
    const dt = curr.time - prev.time;
    if (!(dt > 0 && dt <= 2.0)) continue;
    const segmentDistance = distanceMeters(prev, curr);
    if (segmentDistance / dt > 120) continue;

    const p = latLonToLocalMeters(prev, origin);
    const q = latLonToLocalMeters(curr, origin);
    const pRel = { x: p.x - a.x, y: p.y - a.y };
    const qRel = { x: q.x - a.x, y: q.y - a.y };
    const sidePrev = cross2(line, pRel);
    const sideCurr = cross2(line, qRel);
    if (sidePrev === 0 || sideCurr === 0 || sidePrev * sideCurr >= 0) continue;

    const fraction = sidePrev / (sidePrev - sideCurr);
    if (!(fraction >= 0 && fraction <= 1)) continue;
    const intersection = {
      x: p.x + (q.x - p.x) * fraction,
      y: p.y + (q.y - p.y) * fraction
    };
    const u = ((intersection.x - a.x) * line.x + (intersection.y - a.y) * line.y) / lineLengthSq;
    if (u < 0 || u > 1) continue;

    const speed = prev.speed + (curr.speed - prev.speed) * fraction;
    if (speed < 10) continue;
    candidates.push({
      time: prev.time + dt * fraction,
      gpsTime: Number.isFinite(prev.gpsTime) && Number.isFinite(curr.gpsTime)
        ? prev.gpsTime + (curr.gpsTime - prev.gpsTime) * fraction
        : NaN,
      lat: prev.lat + (curr.lat - prev.lat) * fraction,
      lon: prev.lon + (curr.lon - prev.lon) * fraction,
      direction: sideCurr > sidePrev ? 1 : -1,
      speed
    });
  }

  if (!candidates.length) {
    renderGpsLapResults([], []);
    setGpsLapStatus('선 통과를 찾지 못했습니다. 라인을 트랙 폭 전체에 걸쳐 다시 설정하십시오.', 'warn');
    return;
  }

  // 첫 통과 방향을 정방향으로 삼고 반대 방향 통과와 근접 중복 검출을 제거합니다.
  const forwardDirection = candidates[0].direction;
  const minLapSeconds = Math.max(5, Number(gpsLapMinTime?.value) || 20);
  const crossings = [];
  candidates.forEach(candidate => {
    if (candidate.direction !== forwardDirection) return;
    const previous = crossings[crossings.length - 1];
    if (previous && crossingElapsedSeconds(previous, candidate).duration < minLapSeconds) return;
    crossings.push(candidate);
  });

  const laps = [];
  for (let i = 1; i < crossings.length; i++) {
    const elapsed = crossingElapsedSeconds(crossings[i - 1], crossings[i]);
    if (elapsed.duration < minLapSeconds) continue;
    laps.push({
      number: laps.length + 1,
      duration: elapsed.duration,
      timeBasis: elapsed.basis,
      startTime: crossings[i - 1].time,
      endTime: crossings[i].time,
      startGpsTime: crossings[i - 1].gpsTime,
      endGpsTime: crossings[i].gpsTime,
      startLat: crossings[i - 1].lat,
      startLon: crossings[i - 1].lon,
      endLat: crossings[i].lat,
      endLon: crossings[i].lon
    });
  }

  laps.forEach(lap => {
    lap.distanceMeters = calculateGpsLapDistance(lap);
  });

  renderGpsLapResults(crossings, laps);
  setGpsLapStatus(laps.length ? `${laps.length}개 랩 계산 완료 · 통과 시각 선형 보간 적용` : '첫 통과만 검출되어 완성된 랩이 없습니다.', laps.length ? 'ok' : 'warn');
}

function clearGpsLapAnalysis(removeSaved = false) {
  gpsLapSelectionActive = false;
  gpsFinishPoints = [];
  clearGpsCheckpoints();
  if (gpsGoProSourceType || gpsGoProMatched) closeGoProVideo();
  closeYouTubeDialog();
  gpsLapResults = [];
  gpsSelectedLapIndex = -1;
  gpsSelectedLapIndices = [];
  gpsLapRouteLines = [];
  clearGpsCompareMarkers();
  if (gpsFinishLine && gpsMap) gpsMap.removeLayer(gpsFinishLine);
  gpsFinishLine = null;
  if (gpsFinishPreviewLine && gpsMap) gpsMap.removeLayer(gpsFinishPreviewLine);
  gpsFinishPreviewLine = null;
  gpsFinishMarkers = [];
  if (gpsFinishEndpointLayer) gpsFinishEndpointLayer.clearLayers();
  if (gpsLapCrossingLayer) gpsLapCrossingLayer.clearLayers();
  if (gpsLapRouteLayer) gpsLapRouteLayer.clearLayers();
  if (gpsRouteLine) gpsRouteLine.setStyle({ opacity: 0.8, weight: 5 });
  if (gpsLapMapLegend) {
    gpsLapMapLegend.hidden = true;
    gpsLapMapLegend.innerHTML = '';
  }
  if (gpsFullscreenLapTimes) {
    gpsFullscreenLapTimes.hidden = true;
    gpsFullscreenLapTimes.innerHTML = '';
  }
  updateGpsCursorLapColor(Number(scrollBar?.value));
  if (tabGps?.classList.contains('active') && activeSampledData.length) {
    syncGpsTimelineRange(currentStartSec, currentEndSec, scrollBar.value);
  }
  if (gpsLapSetLine) gpsLapSetLine.classList.remove('active');
  if (gpsLapClear) gpsLapClear.disabled = true;
  if (gpsMap) gpsMap.getContainer().classList.remove('gps-lap-selecting');
  if (gpsLapCount) gpsLapCount.textContent = '0 LAPS';
  if (gpsLapBestTime) gpsLapBestTime.textContent = '--:--.---';
  if (gpsLapAverageDistance) gpsLapAverageDistance.textContent = '--.- m';
  if (gpsLapFixSummary) gpsLapFixSummary.textContent = '피니시 라인을 설정하지 않았습니다.';
  if (gpsLapList) gpsLapList.innerHTML = '<div class="gps-lap-empty">지도에서 피니시 라인의 양 끝을 클릭하면 자동으로 랩을 계산합니다.</div>';
  setGpsLapStatus('지도에서 라인 양 끝을 차례로 선택하십시오.');
  updateGpsVideoControlAvailability();
  if (removeSaved) removeSavedGpsFixedLines();
}

function beginGpsFinishLineSelection() {
  if (!gpsMap || gpsLapPoints.length < 2) {
    setGpsLapStatus('먼저 GPS 데이터가 포함된 CSV를 불러오십시오.', 'warn');
    return;
  }
  if (gpsFinishPoints.length === 2 && !verifyGpsFixedLinesPassword('피니시 라인을 다시 설정')) return;
  clearGpsLapAnalysis(true);
  gpsLapSelectionActive = true;
  gpsLapSetLine?.classList.add('active');
  gpsMap.getContainer().classList.add('gps-lap-selecting');
  setGpsLapStatus('피니시 라인의 첫 번째 끝점을 클릭하십시오.', 'warn');
}

function cancelGpsFinishLineSelection() {
  if (!gpsLapSelectionActive) return false;
  gpsLapSelectionActive = false;
  gpsFinishPoints = [];
  gpsLapSetLine?.classList.remove('active');
  gpsMap?.getContainer().classList.remove('gps-lap-selecting');
  if (gpsFinishPreviewLine && gpsMap) gpsMap.removeLayer(gpsFinishPreviewLine);
  gpsFinishPreviewLine = null;
  gpsFinishMarkers = [];
  gpsFinishEndpointLayer?.clearLayers();
  setGpsLapStatus('피니시 라인 설정을 취소했습니다. 다시 설정 버튼을 눌러 시작하십시오.');
  updateGpsVideoControlAvailability();
  return true;
}

function handleGpsLapMapClick(event) {
  if (!gpsLapSelectionActive) return;
  gpsFinishPoints.push({ lat: event.latlng.lat, lon: event.latlng.lng });
  if (gpsFinishPoints.length === 1) {
    drawGpsFinishFirstPoint();
    setGpsLapStatus('이제 피니시 라인의 반대쪽 끝점을 클릭하십시오.', 'warn');
    return;
  }

  gpsLapSelectionActive = false;
  gpsLapSetLine?.classList.remove('active');
  gpsMap.getContainer().classList.remove('gps-lap-selecting');
  if (gpsLapClear) gpsLapClear.disabled = false;
  if (gpsFinishPreviewLine) gpsMap.removeLayer(gpsFinishPreviewLine);
  gpsFinishPreviewLine = null;
  drawGpsFinishLine();
  if (gpsCheckpointClear) gpsCheckpointClear.disabled = false;
  saveGpsFixedLines();
  calculateGpsLaps();
  updateGpsVideoControlAvailability();
}

// Leaflet Map Initialization
// 지도 확대 한도를 크게 늘려서(줌 22까지) 일반 도로 폭 안에서도 GPS 포인트가
// 어느 위치(차선/갓길 등)에 찍혔는지 구분할 수 있도록 합니다. 타일 자체의
// 최대 해상도(maxNativeZoom)를 넘어가면 Leaflet이 남은 배율만큼 타일을
// 확대(오버줌)해서 보여줍니다 — 화질은 약간 흐려지지만 위치 판독에는 충분합니다.
const GPS_MAP_MAX_ZOOM = 22;

function initGpsMap() {
  if (gpsMap) return;
  gpsMap = L.map('gps-map', {
    zoomControl: false,
    maxZoom: GPS_MAP_MAX_ZOOM
  }).setView([36.5, 127.8], 7);

  L.control.zoom({ position: 'bottomright' }).addTo(gpsMap);

  // 그래픽(다크 벡터 스타일) 지도 레이어 — 기존 기본 지도
  gpsGraphicLayer = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    attribution: '&copy; CartoDB',
    maxZoom: GPS_MAP_MAX_ZOOM,
    maxNativeZoom: 20
  });

  // 국내 위성 지도 레이어 (국토교통부 공간정보 오픈플랫폼 VWorld).
  // 인증키는 운영 Vercel 도메인으로 제한되어 있어 다른 사이트에서는 사용할 수 없습니다.
  gpsSatelliteLayer = L.tileLayer('https://api.vworld.kr/req/wmts/1.0.0/FA347C96-2846-3D64-8855-29ED001264B6/Satellite/{z}/{y}/{x}.jpeg', {
    attribution: '영상지도 &copy; 국토교통부 VWorld',
    maxZoom: GPS_MAP_MAX_ZOOM,
    maxNativeZoom: 19
  });

  // 마지막으로 선택한 지도 모드 기억 (브라우저별 로컬 저장)
  const savedMode = (() => {
    try { return localStorage.getItem('nssur_gps_map_mode'); } catch (err) { return null; }
  })();
  currentGpsLayerMode = savedMode === 'satellite' ? 'satellite' : 'graphic';
  (currentGpsLayerMode === 'satellite' ? gpsSatelliteLayer : gpsGraphicLayer).addTo(gpsMap);

  // 위성 ↔ 그래픽 지도 전환 커스텀 컨트롤 버튼
  // 우측 상단은 조향각 위젯이 쓰므로 지도 전환 버튼은 좌측 상단에 둡니다.
  const MapModeToggleControl = L.Control.extend({
    options: { position: 'topleft' },
    onAdd: function () {
      const container = L.DomUtil.create('div', 'map-mode-toggle');
      const button = L.DomUtil.create('a', 'map-mode-toggle-btn', container);
      button.href = '#';
      button.title = '위성 지도 / 그래픽 지도 전환';

      const refreshLabel = () => {
        button.innerHTML = currentGpsLayerMode === 'satellite' ? '🗺️ 그래픽 지도' : '🛰️ 위성 지도';
      };
      refreshLabel();

      L.DomEvent.disableClickPropagation(container);
      L.DomEvent.on(button, 'click', (e) => {
        L.DomEvent.preventDefault(e);
        if (currentGpsLayerMode === 'graphic') {
          gpsMap.removeLayer(gpsGraphicLayer);
          gpsSatelliteLayer.addTo(gpsMap);
          currentGpsLayerMode = 'satellite';
        } else {
          gpsMap.removeLayer(gpsSatelliteLayer);
          gpsGraphicLayer.addTo(gpsMap);
          currentGpsLayerMode = 'graphic';
        }
        try { localStorage.setItem('nssur_gps_map_mode', currentGpsLayerMode); } catch (err) { /* ignore */ }
        refreshLabel();
      });

      return container;
    }
  });
  new MapModeToggleControl().addTo(gpsMap);

  gpsRouteLine = L.polyline([], {
    color: '#f97316',
    weight: 5,
    opacity: 0.8,
    interactive: true
  }).addTo(gpsMap);
  gpsFinishEndpointLayer = L.layerGroup().addTo(gpsMap);
  gpsLapRouteLayer = L.layerGroup().addTo(gpsMap);
  gpsLapCrossingLayer = L.layerGroup().addTo(gpsMap);
  gpsCheckpointLayer = L.layerGroup().addTo(gpsMap);
  gpsCheckpointDraftLayer = L.layerGroup().addTo(gpsMap);
  gpsMap.on('click', handleGpsLapMapClick);
  gpsMap.on('click', handleGpsCheckpointMapClick);
  gpsMap.on('mousemove', updateGpsFinishPreview);
  gpsMap.on('mousemove', updateGpsCheckpointPreview);
  gpsMap.on('zoom', updateGpsCursorScale);
}

gpsLapSetLine?.addEventListener('click', beginGpsFinishLineSelection);
gpsLapClear?.addEventListener('click', () => clearGpsLapAnalysis(true));
gpsCheckpointAdd?.addEventListener('click', beginGpsCheckpointSelection);
gpsSectorToggle?.addEventListener('click', toggleGpsSectorOverlay);
gpsSectorOverlayClose?.addEventListener('click', closeGpsSectorOverlay);
gpsCheckpointClear?.addEventListener('click', () => {
  if (!verifyGpsFixedLinesPassword('피니시라인과 체크포인트를 초기화')) return;
  if (!window.confirm('피니시라인과 체크포인트 2개를 모두 삭제하시겠습니까?')) return;
  clearGpsLapAnalysis(true);
  setGpsLapStatus('피니시라인과 체크포인트를 모두 초기화했습니다.');
});
gpsLapMinTime?.addEventListener('change', () => {
  const clamped = Math.max(5, Math.min(600, Number(gpsLapMinTime.value) || 20));
  gpsLapMinTime.value = String(clamped);
  if (gpsFinishPoints.length === 2) calculateGpsLaps();
});

gpsLapMapLegend?.addEventListener('click', event => {
  const button = event.target.closest('[data-lap-view]');
  if (!button) return;
  const index = button.dataset.lapView === 'all' ? -1 : Number(button.dataset.lapView);
  selectGpsLapView(index);
});

gpsFullscreenLapTimes?.addEventListener('click', event => {
  const button = event.target.closest('[data-lap-panel-view]');
  if (!button) return;
  const index = button.dataset.lapPanelView === 'all' ? -1 : Number(button.dataset.lapPanelView);
  selectGpsLapView(index);
});

gpsFullscreenDetailToggle?.addEventListener('click', () => {
  const stage = gpsFullscreenDetailToggle.closest('.gps-map-stage');
  const open = !gpsFullscreenDetail.classList.contains('open');
  if (open) closeGpsSectorOverlay();
  gpsFullscreenDetail.classList.toggle('open', open);
  stage?.classList.toggle('gps-detail-open', open);
  gpsFullscreenDetailToggle.textContent = open ? '상세정보 닫기 ›' : '상세정보 ›';
  if (open) {
    ensureGpsDetailCharts();
    if (gpsSelectedLapIndices.length > 1) rebuildGpsDetailChartsForSelection();
    updateGpsDetailChartRange(Number(scrollBar.min), Number(scrollBar.max));
    updateGpsDetailCursors(Number(scrollBar.value));
  }
  setTimeout(() => {
    gpsMap?.invalidateSize();
    refitGpsMapToCurrentLapView();
    gpsDetailCharts.forEach(chart => chart.resize());
    updateGpsDetailCursors(Number(scrollBar.value));
  }, 100);
});

function setGpsAppFullscreen(active) {
  const card = gpsMapFullscreen.closest('.gps-map-card');
  if (!card) return;
  card.classList.toggle('gps-map-fullscreen-fallback', active);
  card.classList.toggle('is-gps-fullscreen', active);
  document.body.classList.toggle('gps-map-fullscreen-open', active);
  gpsMapFullscreen.textContent = active ? '✕ 전체화면 종료' : '⛶ 전체화면';
  if (active) refreshGpsFullscreenOverlays();
  else closeGpsFullscreenDetail();
  setTimeout(() => {
    gpsMap?.invalidateSize();
    refitGpsMapToCurrentLapView();
    updateGpsFullscreenTimelineVisual();
  }, 80);
}

gpsMapFullscreen?.addEventListener('click', () => {
  const card = gpsMapFullscreen.closest('.gps-map-card');
  setGpsAppFullscreen(!card?.classList.contains('gps-map-fullscreen-fallback'));
});

document.addEventListener('keydown', event => {
  if (event.key !== 'Escape') return;
  if (cancelGpsCheckpointSelection()) {
    event.preventDefault();
    return;
  }
  if (cancelGpsFinishLineSelection()) {
    event.preventDefault();
    return;
  }
  if (gpsSectorOverlay && !gpsSectorOverlay.hidden) {
    closeGpsSectorOverlay();
    event.preventDefault();
    return;
  }
  const card = gpsMapFullscreen?.closest('.gps-map-card');
  if (card?.classList.contains('gps-map-fullscreen-fallback')) setGpsAppFullscreen(false);
});

document.addEventListener('fullscreenchange', () => {
  const card = gpsMapFullscreen?.closest('.gps-map-card');
  const active = document.fullscreenElement === card;
  card?.classList.toggle('is-gps-fullscreen', active);
  if (!active) closeGpsFullscreenDetail();
  if (gpsMapFullscreen) gpsMapFullscreen.textContent = active ? '✕ 전체화면 종료' : '⛶ 전체화면';
  if (active) refreshGpsFullscreenOverlays();
  setTimeout(() => {
    gpsMap?.invalidateSize();
    refitGpsMapToCurrentLapView();
    updateGpsFullscreenTimelineVisual();
  }, 80);
});
gpsLapList?.addEventListener('click', event => {
  const summary = event.target.closest('summary');
  if (summary && gpsLapList.contains(summary)) {
    const details = summary.closest('.gps-lap-row');
    requestAnimationFrame(() => {
      if (!details || !details.open) return;
      const hiddenBelow = details.getBoundingClientRect().bottom - gpsLapList.getBoundingClientRect().bottom;
      if (hiddenBelow > 0) gpsLapList.scrollTop += hiddenBelow;
    });
  }

  const row = event.target.closest('[data-lap-time]');
  if (!row) return;
  const targetTime = Number(row.dataset.lapTime);
  if (Number.isFinite(targetTime)) {
    setGpsPlayback(false);
    gpsPlaybackCursorSec = targetTime;
    updateGpsCursorAtTime(targetTime);
  }
});

// Theme Toggle Event Listener
btnThemeToggle.addEventListener('click', () => {
  document.body.classList.toggle('dark-mode');
  updateChartsTheme();
});

// Real-time theme updates inside ChartJS instances without destroying them
function updateChartsTheme() {
  const isDark = document.body.classList.contains('dark-mode');
  const gridColor = isDark ? 'rgba(255, 255, 255, 0.03)' : 'rgba(0, 0, 0, 0.04)';
  const tickColor = isDark ? '#8c96a8' : '#64748b';

  const targetCharts = [
    chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
    diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
    chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
  ];
  targetCharts.forEach(chart => {
    if (!chart) return;

    // 다크모드일 때 눈 피로를 낮춰주는 차분한 파스텔톤으로 선 색상 변경
    updateDatasetColors(chart, isDark);

    if (chart.options.scales) {
      if (chart.options.scales.x) {
        chart.options.scales.x.grid.color = gridColor;
        if (chart.options.scales.x.ticks && chart.options.scales.x.ticks.display !== false) {
          chart.options.scales.x.ticks.color = tickColor;
        }
      }
      if (chart.options.scales.y) {
        if (chart === chartSteering || chart === diagChartSteering) {
          chart.options.scales.y.grid.color = (context) => (context.value === 0 ? '#ff2d55' : gridColor);
        } else {
          chart.options.scales.y.grid.color = gridColor;
        }
        chart.options.scales.y.ticks.color = tickColor;
      }
      if (chart.options.scales.ySpeed) {
        chart.options.scales.ySpeed.ticks.color = tickColor;
        chart.options.scales.ySpeed.title.color = tickColor;
      }
    }

    if (chart.options.plugins && chart.options.plugins.legend && chart.options.plugins.legend.labels) {
      chart.options.plugins.legend.labels.color = tickColor;
    }

    chart.update('none');
  });

  clearAllDomCursors();
}

function updateDatasetColors(chart, isDark) {
  if (!chart) return;
  const id = chart.canvas.id;
  
  chart.data.datasets.forEach((dataset, idx) => {
    if (id === 'chart-ground-speed' || id === 'chart-vehicle-speed') {
      if (idx === 0) dataset.borderColor = isDark ? '#ffb07c' : '#f97316';
      if (idx === 1) dataset.borderColor = isDark ? '#74b9ff' : '#2563eb';
    } else if (id === 'chart-engine-rpm') {
      dataset.borderColor = isDark ? '#ff7675' : '#dc2626';
    } else if (id === 'chart-vehicle-gear') {
      dataset.borderColor = isDark ? '#74b9ff' : '#2563eb';
    } else if (id === 'chart-steering-angle') {
      if (idx === 0) dataset.borderColor = isDark ? '#fd79a8' : '#db2777';
      if (idx === 1) dataset.borderColor = isDark ? '#4ade80' : '#22c55e';
      if (idx === 2) dataset.borderColor = isDark ? '#74b9ff' : '#2563eb';
    } else if (id === 'chart-throttle-brake' || id === 'diag-chart-throttle-brake') {
      if (idx === 0) dataset.borderColor = isDark ? '#55efc4' : '#16a34a'; // Throttle
      if (idx === 1) dataset.borderColor = isDark ? '#ff7675' : '#dc2626'; // Brake
    } else if (id === 'diag-chart-steering') {
      dataset.borderColor = isDark ? '#fd79a8' : '#db2777';
    } else if (id === 'chart-sus-fl') {
      dataset.borderColor = isDark ? '#fd79a8' : '#db2777';
    } else if (id === 'chart-sus-rl') {
      dataset.borderColor = isDark ? '#81ecec' : '#06b6d4';
    } else if (id === 'chart-sus-fr') {
      dataset.borderColor = isDark ? '#ff7675' : '#dc2626';
    } else if (id === 'chart-sus-rr') {
      dataset.borderColor = isDark ? '#74b9ff' : '#2563eb';
    } else if (id === 'chart-coolant-oil') {
      if (idx === 0) dataset.borderColor = isDark ? '#74b9ff' : '#2563eb';
      if (idx === 1) dataset.borderColor = isDark ? '#ffb07c' : '#f97316';
      if (idx === 2) dataset.borderColor = isDark ? '#81ecec' : '#06b6d4';
    } else if (id === 'chart-intake-ecu') {
      dataset.borderColor = idx === 0
        ? (isDark ? '#55efc4' : '#16a34a')
        : (isDark ? '#fd79a8' : '#db2777');
    } else if (id === 'chart-imu-accel' || id === 'chart-imu-gyro') {
      const light = ['#f97316', '#2563eb', '#16a34a'];
      const dark = ['#ffb07c', '#74b9ff', '#55efc4'];
      dataset.borderColor = (isDark ? dark : light)[idx] || dataset.borderColor;
    }
  });
}

// Tab Switching Event Bindings
tabGeneral.addEventListener('click', () => switchTab('general'));
tabDiagnostics.addEventListener('click', () => switchTab('diag'));
if (tabGps) {
  tabGps.addEventListener('click', () => switchTab('gps'));
}
if (tabTemperature) {
  tabTemperature.addEventListener('click', () => switchTab('temperature'));
}
if (tabRealtime) {
  tabRealtime.addEventListener('click', () => switchTab('realtime'));
}
if (tabHelp) {
  tabHelp.addEventListener('click', () => switchTab('help'));
}

pageHelp?.addEventListener('click', event => {
  const button = event.target.closest('[data-help-tab]');
  if (button) switchTab(button.dataset.helpTab);
});

// Keyboard shortcuts: number row and numeric keypad 1–6 switch pages.
document.addEventListener('keydown', (event) => {
  const target = event.target;
  const isTyping = target instanceof HTMLInputElement ||
    target instanceof HTMLTextAreaElement ||
    target instanceof HTMLSelectElement ||
    target?.isContentEditable;

  if (isTyping || event.ctrlKey || event.metaKey || event.altKey) return;

  const pageByKey = {
    Digit1: 'general',
    Numpad1: 'general',
    Digit2: 'diag',
    Numpad2: 'diag',
    Digit3: 'gps',
    Numpad3: 'gps',
    Digit4: 'temperature',
    Numpad4: 'temperature',
    Digit5: 'realtime',
    Numpad5: 'realtime',
    Digit6: 'help',
    Numpad6: 'help'
  };
  const mode = pageByKey[event.code];
  if (!mode) return;

  event.preventDefault();
  switchTab(mode);
});

function switchTab(mode) {
  if (mode !== 'gps' && gpsPlaybackActive) setGpsPlayback(false);

  // Remove active from all tabs and pages
  tabGeneral.classList.remove('active');
  tabDiagnostics.classList.remove('active');
  if (tabGps) tabGps.classList.remove('active');
  if (tabTemperature) tabTemperature.classList.remove('active');
  if (tabRealtime) tabRealtime.classList.remove('active');
  if (tabHelp) tabHelp.classList.remove('active');

  pageGeneral.classList.remove('active');
  pageDiagnostics.classList.remove('active');
  if (pageGps) pageGps.classList.remove('active');
  if (pageTemperature) pageTemperature.classList.remove('active');
  if (pageRealtime) pageRealtime.classList.remove('active');
  if (pageHelp) pageHelp.classList.remove('active');

  clearAllDomCursors();

  // 실시간과 도움말 페이지는 로그 재생용 타임라인이 필요 없으므로 숨깁니다.
  if (timelineNavigator) {
    timelineNavigator.style.display = (mode === 'realtime' || mode === 'help') ? 'none' : '';
  }

  if (mode === 'help') {
    if (tabHelp) tabHelp.classList.add('active');
    if (pageHelp) {
      pageHelp.classList.add('active');
      pageHelp.scrollTop = 0;
    }
    return;
  }

  if (mode === 'realtime') {
    if (tabRealtime) tabRealtime.classList.add('active');
    if (pageRealtime) pageRealtime.classList.add('active');
    setTimeout(() => {
      Object.values((typeof rtState !== 'undefined' && rtState.cards) || {}).forEach(c => {
        if (c.chart) { c.chart.resize(); c.chart.update('none'); }
      });
      if (typeof rtScheduleRender === 'function') rtScheduleRender();
    }, 50);
    return;
  }

  if (mode === 'general') {
    tabGeneral.classList.add('active');
    pageGeneral.classList.add('active');
    if (lblScrollType) {
      lblScrollType.textContent = '📊 그래프 좌우 스크롤:';
    }
    setTimeout(() => {
      [chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake].forEach(c => { 
        if (c) {
          c.resize();
          c.update(); // 풀 업데이트로 데이터셋 좌표 갱신 강제
        }
      });
      // 차트 리사이즈 및 갱신이 완료된 픽셀 기반 위치로 세로선과 교차점 점들 복원
      drawCssIntersectionDots(currentCursorIndex);
    }, 50);
  } else if (mode === 'diag') {
    tabDiagnostics.classList.add('active');
    pageDiagnostics.classList.add('active');
    if (lblScrollType) {
      lblScrollType.textContent = '📊 그래프 좌우 스크롤:';
    }
    setTimeout(() => {
      [diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR].forEach(c => {
        if (c) {
          c.resize();
          c.update(); // 풀 업데이트로 데이터셋 좌표 갱신 강제
        }
      });
      // 차트 리사이즈 및 갱신이 완료된 픽셀 기반 위치로 세로선과 교차점 점들 복원
      drawCssIntersectionDots(currentCursorIndex);
    }, 50);
  } else if (mode === 'gps') {
    if (tabGps) tabGps.classList.add('active');
    if (pageGps) pageGps.classList.add('active');
    if (lblScrollType) {
      lblScrollType.textContent = '📍 실시간 주행 시점 슬라이더:';
    }
    // Invalidate map and charts after their previously hidden page becomes visible.
    setTimeout(() => {
      if (gpsMap) {
        gpsMap.invalidateSize();
        // GPS 페이지 진입 시점에 즉시 현재 커서 위치로 핀 갱신
        const row = activeSampledData[currentCursorIndex];
        if (row) {
          updateNumericDisplays(row);
        }
        // 전체 주행 경로가 지도 화면에 딱 들어맞도록 줌 및 정렬 강제 갱신
        if (gpsRouteLine && gpsRouteLine.getLatLngs().length > 0) {
          gpsMap.fitBounds(gpsRouteLine.getBounds(), { padding: [30, 30] });
        }
      }
      [chartImuAccel, chartImuGyro].forEach(c => {
        if (c) { c.resize(); c.update('none'); }
      });
      drawCssIntersectionDots(currentCursorIndex);
    }, 50);
  } else if (mode === 'temperature') {
    if (tabTemperature) tabTemperature.classList.add('active');
    if (pageTemperature) pageTemperature.classList.add('active');
    if (lblScrollType) {
      lblScrollType.textContent = '🌡️ 온도 그래프 좌우 스크롤:';
    }
    setTimeout(() => {
      [chartCoolantOil, chartIntakeEcu].forEach(c => {
        if (c) {
          c.resize();
          c.update();
        }
      });
      drawCssIntersectionDots(currentCursorIndex);
    }, 50);
  }

  // 탭 전환에 맞춰 스크롤바 속성(스크러버 vs 패닝) 및 활성화 여부 즉시 갱신
  applyZoomRange(currentStartSec, currentEndSec);
}

// Clear all absolute overlay cursor dots and lines on resize/reload
function clearAllDomCursors() {
  const dots = document.querySelectorAll('.visual-cursor-dot');
  dots.forEach(d => d.style.display = 'none');
  const lines = document.querySelectorAll('.motec-cursor-line');
  lines.forEach(l => l.style.display = 'none');
}

// ==================== [헤더 CSV 열기 버튼 연결] ====================
if (csvUploadInput) {
  csvUploadInput.addEventListener('change', (e) => {
    if (e.target.files.length > 0) {
      handleFile(e.target.files[0]);
      csvUploadInput.value = ''; // 같은 파일 다시 열기 가능하게 초기화
    }
  });
}

// 전체 화면 드래그앤드롭 지원 및 시각 오버레이 피드백
const dragOverlay = document.getElementById('drag-drop-overlay');

window.addEventListener('dragenter', (e) => {
  if (e.dataTransfer.types.includes('Files')) {
    e.preventDefault();
    if (dragOverlay) dragOverlay.classList.add('active');
  }
});

window.addEventListener('dragleave', (e) => {
  if (e.clientX <= 0 || e.clientY <= 0 || e.clientX >= window.innerWidth || e.clientY >= window.innerHeight) {
    if (dragOverlay) dragOverlay.classList.remove('active');
  }
});

window.addEventListener('dragover', (e) => {
  if (e.dataTransfer.types.includes('Files')) {
    e.preventDefault();
  }
});

window.addEventListener('drop', (e) => {
  if (e.dataTransfer.types.includes('Files')) {
    e.preventDefault();
    if (dragOverlay) dragOverlay.classList.remove('active');
    if (e.dataTransfer.files.length > 0) {
      handleFile(e.dataTransfer.files[0]);
    }
  }
});

// ==================== [초고속 60fps 휠/핀치 줌 가로채기 및 쓰로틀링 연동] ====================
let zoomPending = false;
document.addEventListener('wheel', (e) => {
  // 시간축 확대/축소는 실제 그래프가 그려진 canvas 위에서만 동작합니다.
  // 카드, 지도, 수치, 랩 목록의 세로 스크롤은 가로채지 않습니다.
  const chartCanvas = e.target.closest('canvas');
  if (!chartCanvas || !chartCanvas.closest('.canvas-holder, .canvas-holder-sub')) return;
  if (globalData.length === 0 || totalDurationSec <= 0) return;

  // On chart pages the timeline slider stores the viewport start, not the
  // selected cursor time. Anchor zoom to the actual cursor so its horizontal
  // position stays fixed while the surrounding time range changes.
  const cursorRow = activeSampledData[currentCursorIndex];
  let targetTime = cursorRow ? Number(cursorRow.time_sec) : NaN;
  if (tabGps && tabGps.classList.contains('active')) {
    const gpsCursorTime = Number(scrollBar.value);
    if (Number.isFinite(gpsCursorTime)) targetTime = gpsCursorTime;
  }
  if (!Number.isFinite(targetTime) || targetTime < currentStartSec || targetTime > currentEndSec) {
    targetTime = (currentStartSec + currentEndSec) / 2;
  }

  e.preventDefault(); // 스크롤바 바운스 차단

  if (zoomPending) return;
  zoomPending = true;

  // 브라우저 60Hz 렌더링 프레임 단위로 가속화 스케일 연산 바인딩
  requestAnimationFrame(() => {
    const currentSpan = currentEndSec - currentStartSec;
    const zoomFactor = 0.08; // 8% 줌 스케일로 더 촘촘하고 차분한 미세 탐색 지원
    let newSpan = currentSpan;

    if (e.deltaY < 0) {
      newSpan = currentSpan * (1 - zoomFactor);
      if (newSpan < 2.0) newSpan = 2.0; // 줌인 최소 스팬 2초
    } else {
      newSpan = currentSpan * (1 + zoomFactor);
      if (newSpan > totalDurationSec) newSpan = totalDurationSec; // 줌아웃 최대 스팬
    }

    const ratio = currentSpan > 0 ? (targetTime - currentStartSec) / currentSpan : 0.5;
    let newStart = targetTime - (newSpan * ratio);
    let newEnd = targetTime + (newSpan * (1 - ratio));

    if (newStart < 0) {
      newStart = 0;
      newEnd = Math.min(newSpan, totalDurationSec);
    }
    if (newEnd > totalDurationSec) {
      newEnd = totalDurationSec;
      newStart = Math.max(0, totalDurationSec - newSpan);
    }

    applyZoomRange(newStart, newEnd);
    zoomPending = false;
  });
}, { passive: false });
// =======================================================================================

// Zoom & Scroll Event Bindings
btnApply.addEventListener('click', () => {
  const start = parseFloat(inputStart.value) || 0;
  const end = parseFloat(inputEnd.value) || 10;
  
  limitStartSec = Math.max(0, start);
  limitEndSec = Math.min(totalDurationSec, end);
  if (limitStartSec >= limitEndSec) {
    limitEndSec = Math.min(limitStartSec + 5, totalDurationSec);
  }

  applyZoomRange(limitStartSec, limitEndSec);
});

btnReset.addEventListener('click', () => {
  limitStartSec = 0;
  limitEndSec = totalDurationSec;
  applyZoomRange(0, totalDurationSec);
});

const handleEnterKeyZoom = (e) => {
  if (e.key === 'Enter') {
    btnApply.click();
    e.target.blur();
  }
};
inputStart.addEventListener('keydown', handleEnterKeyZoom);
inputEnd.addEventListener('keydown', handleEnterKeyZoom);

function updateColumnCursorLine(lineId, chart, index) {
  const lineEl = document.getElementById(lineId);
  if (!lineEl) return;

  if (!chart || !chart.chartArea || index === undefined || index === null) {
    lineEl.style.display = 'none';
    return;
  }

  const meta = chart.getDatasetMeta(0);
  if (!meta || meta.hidden) {
    lineEl.style.display = 'none';
    return;
  }

  const point = meta.data[index];
  if (point && !isNaN(point.x)) {
    const canvas = chart.canvas;
    const container = lineEl.parentElement;
    
    // Calculate relative left offset of canvas inside the container, accounting for parent border width
    const rectCanvas = canvas.getBoundingClientRect();
    const rectContainer = container.getBoundingClientRect();
    const borderLeft = parseFloat(window.getComputedStyle(container).borderLeftWidth) || 0;
    const relativeLeft = (rectCanvas.left - rectContainer.left) - borderLeft;

    // Center the 2px-wide cursor line on point.x (subtract 1px for half-width)
    lineEl.style.left = (relativeLeft + point.x - 1) + 'px';
    lineEl.style.display = 'block';
  } else {
    lineEl.style.display = 'none';
  }
}

// HIGH-PERFORMANCE: Places bright circles directly on the intersection points of the chart lines
function drawCssIntersectionDots(index, chartSubset = null) {
  if (globalData.length === 0 || activeSampledData.length === 0) return;

  const targetCharts = chartSubset || [
    chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
    diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
    chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
  ];
  
  targetCharts.forEach(chart => {
    if (!chart || !chart.chartArea) return;
    
    const canvas = chart.canvas;
    const holder = canvas.parentElement;
    
    const existingDots = holder.querySelectorAll('.visual-cursor-dot');
    existingDots.forEach(dot => dot.style.display = 'none');

    chart.data.datasets.forEach((dataset, datasetIndex) => {
      const meta = chart.getDatasetMeta(datasetIndex);
      if (!meta.hidden) {
        // 호버 포인트 매핑 시 현재 X축 구간 내 인덱스 역산으로 올바르게 보정
        const point = meta.data[index];
        if (point && !isNaN(point.x) && !isNaN(point.y)) {
          let dot = holder.querySelector(`.visual-cursor-dot-ds-${datasetIndex}`);
          if (!dot) {
            dot = document.createElement('div');
            dot.className = `visual-cursor-dot visual-cursor-dot-ds-${datasetIndex}`;
            dot.style.position = 'absolute';
            dot.style.width = '10px';
            dot.style.height = '10px';
            dot.style.borderRadius = '50%';
            dot.style.border = '2px solid #ffffff';
            dot.style.pointerEvents = 'none';
            dot.style.zIndex = '12';
            dot.style.transform = 'translate(-50%, -50%)';
            holder.appendChild(dot);
          }
          const color = dataset.borderColor || '#00d2ff';
          dot.style.backgroundColor = color;
          dot.style.boxShadow = `0 0 8px ${color}, 0 0 2px #ffffff`;
          dot.style.display = 'block';
          dot.style.left = point.x + 'px';
          dot.style.top = point.y + 'px';
        }
      }
    });
  });

  // 세로 관통 커서선 위치 업데이트
  updateColumnCursorLine('cursor-line-page1-left', chartSpeed, index);
  updateColumnCursorLine('cursor-line-page1-right', chartSteering, index);
  updateColumnCursorLine('cursor-line-page2-top', diagChartThrottleBrake, index);
  updateColumnCursorLine('cursor-line-page2-bot-left', chartFL, index);
  updateColumnCursorLine('cursor-line-page2-bot-right', chartFR, index);
}

// Panning/Scrolling scrollbar event
let dragSyncPending = false;
let lastDragEvent = null;

function findSampleIndexAtTime(targetTime) {
  let lo = 0;
  let hi = activeSampledData.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (activeSampledData[mid].time_sec < targetTime) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0) {
    const before = activeSampledData[lo - 1];
    const after = activeSampledData[lo];
    if (Math.abs(before.time_sec - targetTime) <= Math.abs(after.time_sec - targetTime)) return lo - 1;
  }
  return lo;
}

function findGlobalIndexAtTime(targetTime) {
  let lo = 0;
  let hi = globalData.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (globalData[mid].time_sec < targetTime) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0) {
    const before = globalData[lo - 1];
    const after = globalData[lo];
    if (Math.abs(before.time_sec - targetTime) <= Math.abs(after.time_sec - targetTime)) return lo - 1;
  }
  return lo;
}

// GPS fixes are recorded less frequently than the 100 Hz telemetry rows. Move
// the map marker continuously between consecutive fixes instead of holding it
// still and jumping whenever a new fix arrives.
function getInterpolatedGpsPosition(targetTime, nearbyIndex) {
  if (!globalData.length || nearbyIndex < 0) return null;

  let floorIndex = nearbyIndex;
  while (floorIndex > 0 && globalData[floorIndex].time_sec > targetTime) floorIndex--;

  const coordsAt = index => {
    const row = globalData[index];
    const lat = convertNmeaToDecimal(row.gps_lat, false);
    const lon = convertNmeaToDecimal(row.gps_lon, true);
    return Number.isFinite(lat) && Number.isFinite(lon) ? { lat, lon } : null;
  };

  let previousIndex = floorIndex;
  let previous = coordsAt(previousIndex);
  while (!previous && previousIndex > 0) previous = coordsAt(--previousIndex);
  if (!previous) return null;

  // Repeated rows carry the last fix forward. Use the first row of that fix as
  // the interpolation start so progress remains linear throughout the interval.
  while (previousIndex > 0) {
    const earlier = coordsAt(previousIndex - 1);
    if (!earlier || earlier.lat !== previous.lat || earlier.lon !== previous.lon) break;
    previousIndex--;
  }

  // Locate the first following row whose GPS fix is actually different.
  let nextIndex = Math.max(floorIndex + 1, previousIndex + 1);
  let next = null;
  while (nextIndex < globalData.length) {
    const candidate = coordsAt(nextIndex);
    if (candidate && (candidate.lat !== previous.lat || candidate.lon !== previous.lon)) {
      next = candidate;
      break;
    }
    nextIndex++;
  }
  if (!next) return previous;

  // Do not invent motion across a long GPS outage.
  const previousTime = globalData[previousIndex].time_sec;
  const nextTime = globalData[nextIndex].time_sec;
  const gap = nextTime - previousTime;
  if (!(gap > 0) || gap > 10) return previous;
  const ratio = Math.max(0, Math.min(1, (targetTime - previousTime) / gap));
  return {
    lat: previous.lat + (next.lat - previous.lat) * ratio,
    lon: previous.lon + (next.lon - previous.lon) * ratio
  };
}

// Chart.js lines are downsampled for performance, but the playback cursor must
// use the exact playback time and the original 100 Hz IMU values.
function drawExactImuCursor(targetTime, row) {
  const specs = [
    [chartImuAccel, ['imu_accel_x_g', 'imu_accel_y_g']],
    [chartImuGyro, ['imu_gyro_x_dps', 'imu_gyro_y_dps', 'imu_gyro_z_dps']]
  ];

  specs.forEach(([chart, keys]) => {
    if (!chart || !chart.chartArea || !chart.scales.x || !chart.scales.y) return;
    const holder = chart.canvas.parentElement;
    holder.querySelectorAll('.visual-cursor-dot').forEach(dot => dot.style.display = 'none');
    const x = chart.scales.x.getPixelForValue(targetTime);

    keys.forEach((key, datasetIndex) => {
      const value = Number(row[key]);
      const meta = chart.getDatasetMeta(datasetIndex);
      if (!Number.isFinite(value) || !Number.isFinite(x) || !meta || meta.hidden) return;
      const y = chart.scales.y.getPixelForValue(value);
      if (!Number.isFinite(y)) return;
      let dot = holder.querySelector(`.visual-cursor-dot-ds-${datasetIndex}`);
      if (!dot) {
        dot = document.createElement('div');
        dot.className = `visual-cursor-dot visual-cursor-dot-ds-${datasetIndex}`;
        Object.assign(dot.style, {
          position: 'absolute', width: '10px', height: '10px', borderRadius: '50%',
          border: '2px solid #ffffff', pointerEvents: 'none', zIndex: '12',
          transform: 'translate(-50%, -50%)'
        });
        holder.appendChild(dot);
      }
      const color = chart.data.datasets[datasetIndex].borderColor || '#00d2ff';
      dot.style.backgroundColor = color;
      dot.style.boxShadow = `0 0 8px ${color}, 0 0 2px #ffffff`;
      dot.style.left = `${x}px`;
      dot.style.top = `${y}px`;
      dot.style.display = 'block';
    });
  });
}

// Read playback cursor values from the same full-resolution filtered arrays
// used to build the IMU chart. Interpolate between adjacent 100 Hz samples so
// a 60 fps cursor follows the filtered curve instead of snapping to a raw row.
function getFilteredImuRowAtTime(row, targetTime, nearbyIndex) {
  if (!gpsImuLpf || !gpsImuLpf.checked || nearbyIndex < 0 ||
      typeof channelValueAt !== 'function') return row;

  let lowerIndex = nearbyIndex;
  while (lowerIndex > 0 && globalData[lowerIndex].time_sec > targetTime) lowerIndex--;
  const upperIndex = Math.min(globalData.length - 1, lowerIndex + 1);
  const lowerTime = globalData[lowerIndex].time_sec;
  const upperTime = globalData[upperIndex].time_sec;
  const ratio = upperTime > lowerTime
    ? Math.max(0, Math.min(1, (targetTime - lowerTime) / (upperTime - lowerTime)))
    : 0;

  const displayRow = Object.create(row);
  const channels = {
    imu_accel_x_g: 'imu_ax',
    imu_accel_y_g: 'imu_ay',
    imu_gyro_x_dps: 'imu_gx',
    imu_gyro_y_dps: 'imu_gy',
    imu_gyro_z_dps: 'imu_gz'
  };
  Object.entries(channels).forEach(([rowKey, channelKey]) => {
    const lower = channelValueAt(channelKey, lowerIndex);
    const upper = channelValueAt(channelKey, upperIndex);
    if (Number.isFinite(lower) && Number.isFinite(upper)) {
      displayRow[rowKey] = lower + (upper - lower) * ratio;
    } else if (Number.isFinite(lower)) {
      displayRow[rowKey] = lower;
    }
  });
  return displayRow;
}

function updateGpsCursorAtTime(targetTime, playbackFrame = false) {
  if (!activeSampledData.length || !Number.isFinite(targetTime)) return;
  const minTime = Number(scrollBar.min) || 0;
  const maxTime = Number(scrollBar.max) || totalDurationSec;
  const clampedTime = Math.max(minTime, Math.min(maxTime, targetTime));
  if (gpsSelectedLapIndices.length > 1) {
    scrollBar.value = clampedTime.toFixed(2);
    if (gpsPlayTime) gpsPlayTime.textContent = `${clampedTime.toFixed(2)} s`;
    if (gpsFullscreenTimeline) {
      gpsFullscreenTimeline.value = clampedTime.toFixed(2);
      updateGpsFullscreenTimelineVisual();
    }
    const primaryLap = gpsLapResults[gpsSelectedLapIndices[0]];
    const primaryTime = primaryLap.startTime + Math.min(clampedTime, primaryLap.duration);
    const primaryIndex = findGlobalIndexAtTime(primaryTime);
    if (gpsFullscreenPlayTime) gpsFullscreenPlayTime.textContent = `${formatGpsClock(gpsClockAtTelemetryTime(primaryTime, primaryIndex))} KST`;
    const primaryRow = primaryIndex >= 0 ? globalData[primaryIndex] : null;
    if (primaryRow) {
      currentCursorIndex = findSampleIndexAtTime(primaryTime);
      const displayRow = getFilteredImuRowAtTime(primaryRow, primaryTime, primaryIndex);
      const gpsPosition = getInterpolatedGpsPosition(primaryTime, primaryIndex);
      updateNumericDisplays(displayRow, gpsPosition, clampedTime);
      updateGpsCompareMarkers(clampedTime);
      updateGpsDetailCursors(clampedTime);
      gpsFullscreenLapTimes?.querySelectorAll('[data-lap-time-row]').forEach(row => {
        row.classList.toggle('active', gpsSelectedLapIndices.includes(Number(row.dataset.lapTimeRow)));
      });
      const live = gpsFullscreenLapTimes?.querySelector('[data-lap-live]');
      if (live) {
        live.textContent = `${gpsSelectedLapIndices.length}개 랩 비교 · ${formatLapTime(clampedTime)}`;
        live.style.color = '#f97316';
      }
      syncGoProVideo(primaryTime, !playbackFrame);
    }
    return;
  }
  currentCursorIndex = findSampleIndexAtTime(clampedTime);
  // Numeric widgets, G meter and map use the original 100 Hz row. Charts keep
  // their 4,500-point series and only move the cursor to the nearest sample.
  const globalIndex = globalData.length ? findGlobalIndexAtTime(clampedTime) : -1;
  const row = globalIndex >= 0 ? globalData[globalIndex] : activeSampledData[currentCursorIndex];
  scrollBar.value = clampedTime.toFixed(2);
  if (gpsPlayTime) gpsPlayTime.textContent = `${clampedTime.toFixed(2)} s`;
  if (gpsFullscreenTimeline) {
    gpsFullscreenTimeline.value = clampedTime.toFixed(2);
    updateGpsFullscreenTimelineVisual();
  }
  if (gpsFullscreenPlayTime) gpsFullscreenPlayTime.textContent = `${formatGpsClock(gpsClockAtTelemetryTime(clampedTime, globalIndex))} KST`;
  if (row) {
    const displayRow = getFilteredImuRowAtTime(row, clampedTime, globalIndex);
    const gpsPosition = getInterpolatedGpsPosition(clampedTime, globalIndex);
    updateNumericDisplays(displayRow, gpsPosition, clampedTime);
    updateGpsCursorLapColor(clampedTime);
    updateGpsDetailCursors(clampedTime);
    drawExactImuCursor(clampedTime, displayRow);
    syncGoProVideo(clampedTime, !playbackFrame);
  }
}

function setGpsPlayback(shouldPlay) {
  const canPlay = shouldPlay && activeSampledData.length > 0 &&
    tabGps && tabGps.classList.contains('active');
  gpsPlaybackActive = Boolean(canPlay);

  if (gpsPlaybackFrame !== null) {
    cancelAnimationFrame(gpsPlaybackFrame);
    gpsPlaybackFrame = null;
  }
  gpsPlaybackLastTimestamp = null;

  if (gpsPlayToggle) {
    gpsPlayToggle.textContent = gpsPlaybackActive ? '❚❚ 일시정지' : '▶ 재생';
    gpsPlayToggle.classList.toggle('playing', gpsPlaybackActive);
  }
  if (gpsFullscreenPlayToggle) {
    gpsFullscreenPlayToggle.textContent = gpsPlaybackActive ? '❚❚ 일시정지' : '▶ 재생';
    gpsFullscreenPlayToggle.classList.toggle('playing', gpsPlaybackActive);
  }
  if (!gpsPlaybackActive) {
    const cursorTime = Number(scrollBar?.value) || gpsPlaybackCursorSec;
    syncGoProVideo(getGoProTargetTelemetryTime(cursorTime), true);
    requestAnimationFrame(updateGpsFullscreenTimelineVisual);
    return;
  }

  let minTime = Number(scrollBar.min) || 0;
  let maxTime = Number(scrollBar.max) || totalDurationSec;
  const videoLapPair = gpsGoProMatched ? getGoProLapPair() : null;
  const selectedSingleLap = gpsSelectedLapIndices.length === 1 ? gpsLapResults[gpsSelectedLapIndices[0]] : null;
  if (videoLapPair) {
    const longerVideoLapDuration = Math.max(
      gpsLapResults[videoLapPair.primaryIndex].duration,
      gpsLapResults[videoLapPair.compareIndex].duration
    );
    const currentValue = Number(scrollBar.value) || 0;
    syncGpsTimelineRange(0, longerVideoLapDuration, Math.min(currentValue, longerVideoLapDuration));
    minTime = 0;
    maxTime = longerVideoLapDuration;
  }
  const playbackEndTime = selectedSingleLap ? Math.min(maxTime, selectedSingleLap.endTime) : maxTime;
  gpsPlaybackCursorSec = Number(scrollBar.value);
  if (!Number.isFinite(gpsPlaybackCursorSec) || gpsPlaybackCursorSec >= playbackEndTime - 0.01) {
    gpsPlaybackCursorSec = minTime;
    updateGpsCursorAtTime(gpsPlaybackCursorSec);
  }
  syncGoProVideo(getGoProTargetTelemetryTime(gpsPlaybackCursorSec), true);

  const playbackStep = timestamp => {
    if (!gpsPlaybackActive) return;
    if (gpsPlaybackLastTimestamp === null) {
      gpsPlaybackLastTimestamp = timestamp;
      gpsPlaybackFrame = requestAnimationFrame(playbackStep);
      return;
    }

    // 랩을 바꾼 직후 영상 탐색·디코딩이 실제 재생 상태가 될 때까지 텔레메트리도
    // 같은 위치에서 기다립니다. 준비 시간만큼 커서가 먼저 출발하는 현상을 막습니다.
    if (isGoProPlaybackWaiting()) {
      gpsPlaybackLastTimestamp = timestamp;
      syncGoProVideo(getGoProTargetTelemetryTime(gpsPlaybackCursorSec), false);
      gpsPlaybackFrame = requestAnimationFrame(playbackStep);
      return;
    }

    // requestAnimationFrame을 약 60fps로 제한합니다. 120Hz 디스플레이에서도
    // 두 프레임마다 한 번만 갱신해 재생 속도와 CPU 사용량을 일정하게 유지합니다.
    const elapsedMs = timestamp - gpsPlaybackLastTimestamp;
    if (elapsedMs < 15) {
      gpsPlaybackFrame = requestAnimationFrame(playbackStep);
      return;
    }
    gpsPlaybackLastTimestamp = timestamp;

    const rate = Number(gpsPlayRate ? gpsPlayRate.value : 1) || 1;
    gpsPlaybackCursorSec += (elapsedMs / 1000) * rate;
    if (gpsPlaybackCursorSec >= playbackEndTime) {
      updateGpsCursorAtTime(playbackEndTime);
      setGpsPlayback(false);
      return;
    }

    updateGpsCursorAtTime(gpsPlaybackCursorSec, true);
    gpsPlaybackFrame = requestAnimationFrame(playbackStep);
  };

  gpsPlaybackFrame = requestAnimationFrame(playbackStep);
}

if (gpsPlayToggle) {
  gpsPlayToggle.addEventListener('click', () => setGpsPlayback(!gpsPlaybackActive));
}
gpsFullscreenPlayToggle?.addEventListener('click', () => setGpsPlayback(!gpsPlaybackActive));
gpsFullscreenPlayRate?.addEventListener('change', () => {
  if (gpsPlayRate) gpsPlayRate.value = gpsFullscreenPlayRate.value;
});
gpsPlayRate?.addEventListener('change', () => {
  if (gpsFullscreenPlayRate) gpsFullscreenPlayRate.value = gpsPlayRate.value;
});
gpsFullscreenTimeline?.addEventListener('input', event => {
  const targetTime = Number(event.target.value);
  if (!Number.isFinite(targetTime)) return;
  setGpsPlayback(false);
  gpsPlaybackCursorSec = targetTime;
  updateGpsCursorAtTime(targetTime);
  updateGpsFullscreenTimelineVisual();
});
window.addEventListener('resize', updateGpsFullscreenTimelineVisual);
document.addEventListener('visibilitychange', () => {
  if (document.hidden && gpsPlaybackActive) setGpsPlayback(false);
});

function closeYouTubeDialog() {
  if (gpsYouTubeDialog?.open) gpsYouTubeDialog.close();
}

function showYouTubeError(message) {
  gpsGoProMatched = false;
  gpsGoProSourceType = '';
  destroyYouTubePlayers();
  gpsGoProPanel?.classList.remove('youtube-source');
  gpsGoProPanel?.closest('.gps-map-stage')?.classList.remove('gps-video-loaded');
  if (gpsGoProPanel) gpsGoProPanel.hidden = false;
  if (gpsGoProStatus) {
    gpsGoProStatus.textContent = message;
    gpsGoProStatus.className = 'error';
  }
}

async function connectYouTubeVideo(rawUrl) {
  const videoId = extractYouTubeVideoId(rawUrl);
  if (!videoId || !/^[A-Za-z0-9_-]{11}$/.test(videoId)) {
    throw new Error('올바른 YouTube 영상 링크를 입력하세요.');
  }
  if (!globalData.length) throw new Error('먼저 CSV를 열어주세요.');

  setGpsPlayback(false);
  closeGoProVideo();
  gpsGoProSourceType = 'youtube';
  gpsYouTubeVideoId = videoId;
  gpsGoProPanel.hidden = false;
  gpsGoProPanel.classList.add('youtube-source');
  gpsGoProStatus.textContent = 'YouTube 제목과 영상 길이를 확인하는 중…';
  gpsGoProStatus.className = '';

  await loadYouTubeIframeApi();
  const primaryMount = ensureYouTubeMount('gps-youtube-player', '.gps-gopro-primary-slot');
  const compareMount = ensureYouTubeMount('gps-youtube-compare-player', '.gps-gopro-compare-slot');
  [gpsYouTubePrimaryPlayer, gpsYouTubeComparePlayer] = await Promise.all([
    createYouTubePlayer(primaryMount, videoId),
    createYouTubePlayer(compareMount, videoId)
  ]);
  const metadata = await waitForYouTubeMetadata(gpsYouTubePrimaryPlayer);
  if (!(metadata.duration > 0)) throw new Error('YouTube 영상 길이를 확인하지 못했습니다. 처리가 끝난 뒤 다시 시도하세요.');
  if (!metadata.title) throw new Error('YouTube 영상 제목을 읽지 못했습니다. 영상 공개 상태를 일부 공개로 설정하세요.');

  const creationDate = parseYouTubeKstStartDate(metadata.title);
  if (!creationDate) {
    throw new Error('제목에서 시작 시각을 찾지 못했습니다. 예: NS26F_2026-08-11_15-13-24.000_KST_GX014229');
  }
  const match = matchGoProToCsv(creationDate, metadata.duration);
  if (!match?.matched) {
    const videoRange = match
      ? `${formatGpsClock(match.videoStart)}~${formatGpsClock(match.videoStart + metadata.duration)} KST`
      : '시간 확인 불가';
    const csvRange = match?.range
      ? `${formatGpsClock(match.range.first.clock)}~${formatGpsClock(match.range.last.clock)} KST`
      : '시간 확인 불가';
    throw new Error(`시간이 겹치지 않아 연결할 수 없습니다. 영상 ${videoRange} · CSV ${csvRange}`);
  }

  gpsGoProTelemetryStartSec = match.telemetryStart;
  gpsGoProMatched = true;
  gpsGoProAudioSlot = 'primary';
  gpsGoProPanel.closest('.gps-map-stage')?.classList.add('gps-video-loaded');
  updateGoProComparisonLayout();
  gpsGoProStatus.textContent = `YouTube · 영상 시작 ${formatGpsClock(match.videoStartClock)} KST · CSV와 ${formatKoreanDuration(match.overlap)} 겹침 · 자동 동기화 완료`;
  gpsGoProStatus.className = 'success';
  window.localStorage?.setItem('nssur-youtube-url', rawUrl);
  syncGoProVideo(Number(scrollBar.value) || 0, true);
  window.setTimeout(() => {
    gpsMap?.invalidateSize();
    refitGpsMapToCurrentLapView();
  }, 100);
}

gpsYouTubeOpen?.addEventListener('click', () => {
  if (!requireGpsFinishLineForVideo()) return;
  const savedUrl = window.localStorage?.getItem('nssur-youtube-url') || '';
  if (gpsYouTubeUrl && !gpsYouTubeUrl.value) gpsYouTubeUrl.value = savedUrl;
  if (typeof gpsYouTubeDialog?.showModal === 'function') gpsYouTubeDialog.showModal();
  else gpsYouTubeDialog?.setAttribute('open', '');
  window.setTimeout(() => gpsYouTubeUrl?.focus(), 0);
});
gpsGoProPrimaryAudio?.addEventListener('click', () => toggleGoProAudio('primary'));
gpsGoProCompareAudio?.addEventListener('click', () => toggleGoProAudio('compare'));
gpsYouTubeUrlClear?.addEventListener('click', () => {
  if (gpsYouTubeUrl) gpsYouTubeUrl.value = '';
  window.localStorage?.removeItem('nssur-youtube-url');
  gpsYouTubeUrl?.focus();
});
gpsGoProOpen?.addEventListener('click', event => requireGpsFinishLineForVideo(event));
gpsYouTubeCancel?.addEventListener('click', closeYouTubeDialog);
gpsYouTubeCancelBottom?.addEventListener('click', closeYouTubeDialog);
gpsYouTubeDialog?.addEventListener('click', event => {
  if (event.target === gpsYouTubeDialog) closeYouTubeDialog();
});
gpsYouTubeForm?.addEventListener('submit', async event => {
  event.preventDefault();
  const rawUrl = gpsYouTubeUrl?.value.trim() || '';
  closeYouTubeDialog();
  try {
    await connectYouTubeVideo(rawUrl);
  } catch (error) {
    showYouTubeError(error.message || 'YouTube 영상을 연결하지 못했습니다.');
  }
});

helpVideoTitleFile?.addEventListener('change', async event => {
  const file = event.target.files?.[0];
  if (!file) return;
  helpVideoTitleStatus.textContent = 'MP4 내부 촬영 시작 시각을 읽는 중…';
  helpVideoTitleStatus.classList.remove('success', 'error');
  helpVideoTitleOutput.value = '';
  helpVideoDescriptionOutput.value = '';
  helpVideoTitleCopy.disabled = true;
  helpVideoDescriptionCopy.disabled = true;
  try {
    const creationDate = await extractMp4CreationDate(file);
    if (!creationDate) throw new Error('MP4 내부 촬영 시작 시각을 찾지 못했습니다. GoPro 원본 MP4인지 확인하세요.');
    const metadata = makeYouTubeUploadMetadata(file, creationDate);
    helpVideoTitleOutput.value = metadata.title;
    helpVideoDescriptionOutput.value = metadata.description;
    helpVideoTitleCopy.disabled = false;
    helpVideoDescriptionCopy.disabled = false;
    helpVideoTitleStatus.textContent = `${file.name} · 영상 시작 ${metadata.startText} · 제목 생성 완료`;
    helpVideoTitleStatus.classList.add('success');
  } catch (error) {
    helpVideoTitleStatus.textContent = error.message || '영상 정보를 읽지 못했습니다.';
    helpVideoTitleStatus.classList.add('error');
  }
});
helpVideoTitleCopy?.addEventListener('click', () => copyHelpVideoText(helpVideoTitleOutput.value, helpVideoTitleCopy, '제목 복사'));
helpVideoDescriptionCopy?.addEventListener('click', () => copyHelpVideoText(helpVideoDescriptionOutput.value, helpVideoDescriptionCopy, '설명 복사'));

gpsGoProClose?.addEventListener('click', closeGoProVideo);
gpsGoProFile?.addEventListener('change', async event => {
  const file = event.target.files?.[0];
  if (!file) return;
  setGpsPlayback(false);
  closeGoProVideo();
  gpsGoProSourceType = 'local';
  gpsGoProPanel?.classList.remove('youtube-source');
  gpsGoProFile.value = '';
  if (!globalData.length) {
    gpsGoProPanel.hidden = false;
    gpsGoProStatus.textContent = '먼저 CSV를 열어주세요.';
    gpsGoProStatus.className = 'error';
    return;
  }
  gpsGoProPanel.hidden = false;
  gpsGoProStatus.textContent = 'MP4 촬영 시각을 확인하는 중…';
  gpsGoProStatus.className = '';
  try {
    const creationDate = await extractMp4CreationDate(file);
    if (!creationDate) throw new Error('MP4 내부 촬영 시각을 찾을 수 없습니다.');
    gpsGoProObjectUrl = URL.createObjectURL(file);
    gpsGoProVideo.src = gpsGoProObjectUrl;
    if (gpsGoProCompareVideo) gpsGoProCompareVideo.src = gpsGoProObjectUrl;
    await new Promise((resolve, reject) => {
      gpsGoProVideo.onloadedmetadata = resolve;
      gpsGoProVideo.onerror = () => reject(new Error('브라우저에서 이 MP4를 재생할 수 없습니다.'));
    });
    const videoDuration = gpsGoProVideo.duration;
    const match = matchGoProToCsv(creationDate, videoDuration);
    if (!match?.matched) {
      gpsGoProMatched = false;
      gpsGoProVideo.removeAttribute('src');
      gpsGoProCompareVideo?.removeAttribute('src');
      URL.revokeObjectURL(gpsGoProObjectUrl);
      gpsGoProObjectUrl = '';
      const videoRange = match
        ? `${formatGpsClock(match.videoStart)}~${formatGpsClock(match.videoStart + videoDuration)} KST`
        : '시간 확인 불가';
      const csvRange = match?.range
        ? `${formatGpsClock(match.range.first.clock)}~${formatGpsClock(match.range.last.clock)} KST`
        : '시간 확인 불가';
      gpsGoProStatus.textContent = `시간이 겹치지 않아 연결할 수 없습니다. 영상 ${videoRange} · CSV ${csvRange}`;
      gpsGoProStatus.className = 'error';
      return;
    }
    gpsGoProTelemetryStartSec = match.telemetryStart;
    gpsGoProMatched = true;
    gpsGoProAudioSlot = 'primary';
    gpsGoProPanel.closest('.gps-map-stage')?.classList.add('gps-video-loaded');
    updateGoProComparisonLayout();
    gpsGoProStatus.textContent = `영상 시작 ${formatGpsClock(match.videoStartClock)} KST · CSV와 ${formatKoreanDuration(match.overlap)} 겹침 · 자동 동기화 완료`;
    gpsGoProStatus.className = 'success';
    syncGoProVideo(Number(scrollBar.value) || 0, true);
    setTimeout(() => {
      gpsMap?.invalidateSize();
      refitGpsMapToCurrentLapView();
    }, 100);
  } catch (error) {
    gpsGoProMatched = false;
    gpsGoProStatus.textContent = error.message || 'MP4 시간 정보를 읽지 못했습니다.';
    gpsGoProStatus.className = 'error';
  }
});
function applyGpsImuLowPassFilter() {
  const keys = ['imu_ax', 'imu_ay', 'imu_gx', 'imu_gy', 'imu_gz'];
  if (!gpsImuLpf || typeof getFilterState !== 'function' ||
      typeof recomputeChannel !== 'function') return;
  const cutoffHz = Number(gpsImuLpfFrequency ? gpsImuLpfFrequency.value : 5) || 5;
  keys.forEach(key => {
    const state = getFilterState(key);
    state.type = gpsImuLpf.checked ? 'butter' : 'none';
    state.params = gpsImuLpf.checked ? { fc: cutoffHz, order: 2 } : {};
    recomputeChannel(key);
  });
  refreshChartsAfterFilter();
  updateGpsCursorAtTime(Number(scrollBar.value) || 0);
}

if (gpsImuLpf) {
  gpsImuLpf.addEventListener('change', applyGpsImuLowPassFilter);
}
if (gpsImuLpfFrequency) {
  gpsImuLpfFrequency.addEventListener('change', () => {
    if (gpsImuLpf && gpsImuLpf.checked) applyGpsImuLowPassFilter();
  });
}

function getImuChartByCanvasId(canvasId) {
  if (canvasId === 'chart-imu-accel') return chartImuAccel;
  if (canvasId === 'chart-imu-gyro') return chartImuGyro;
  return null;
}

function applyImuAxisToggleState(canvasId) {
  const chart = getImuChartByCanvasId(canvasId);
  if (!chart) return;
  document.querySelectorAll(`.imu-axis-toggle[data-chart="${canvasId}"]`).forEach(button => {
    const datasetIndex = Number(button.dataset.dataset);
    const enabled = button.getAttribute('aria-pressed') !== 'false';
    if (Number.isInteger(datasetIndex)) chart.setDatasetVisibility(datasetIndex, enabled);
  });
  chart.update('none');
}

document.querySelectorAll('.imu-axis-toggle').forEach(button => {
  button.addEventListener('click', () => {
    const enabled = button.getAttribute('aria-pressed') !== 'true';
    button.setAttribute('aria-pressed', String(enabled));
    button.classList.toggle('active', enabled);
    applyImuAxisToggleState(button.dataset.chart);
    const cursorTime = Number(scrollBar.value);
    if (Number.isFinite(cursorTime)) updateGpsCursorAtTime(cursorTime);
  });
});

if (scrollBar) {
  document.querySelectorAll('.steering-series-toggle').forEach(button => {
    button.addEventListener('click', () => {
      if (!chartSteering) return;
      const enabled = button.getAttribute('aria-pressed') !== 'true';
      button.setAttribute('aria-pressed', String(enabled));
      button.classList.toggle('active', enabled);
      chartSteering.setDatasetVisibility(Number(button.dataset.dataset), enabled);
      chartSteering.update('none');
    });
  });
  scrollBar.addEventListener('pointerdown', () => setGpsPlayback(false));
}

const handleTimelineScrollDrag = (e) => {
  lastDragEvent = e;
  if (dragSyncPending) return;
  dragSyncPending = true;

  requestAnimationFrame(() => {
    if (!lastDragEvent || globalData.length === 0 || activeSampledData.length === 0) {
      dragSyncPending = false;
      return;
    }
    
    // GPS 페이지 활성화 시: 시간 스크러버(Scrubber)로 동작
    if (tabGps && tabGps.classList.contains('active')) {
      const targetTime = parseFloat(lastDragEvent.target.value);
      if (!isNaN(targetTime)) updateGpsCursorAtTime(targetTime);
    } else {
      // 일반 차트 페이지 활성화 시: 뷰포트 스크롤(Panning)로 동작
      const scrollStart = parseFloat(lastDragEvent.target.value);
      if (!isNaN(scrollStart)) {
        const currentSpan = currentEndSec - currentStartSec;
        const newStart = scrollStart;
        const newEnd = scrollStart + currentSpan;
        applyZoomRange(newStart, newEnd);
      }
    }
    dragSyncPending = false;
  });
};

scrollBar.addEventListener('input', handleTimelineScrollDrag);
scrollBar.addEventListener('change', handleTimelineScrollDrag);

// 커서 위치의 채널 값을 읽습니다. 노이즈 필터가 걸려 있으면 필터 적용값을
// 반환해서 그래프와 숫자 표시가 항상 같은 값을 가리키도록 합니다.
function cursorChannelValue(key, fallback) {
  if (typeof channelValueAt === 'function' && sampleIndices.length) {
    const v = channelValueAt(key, sampleIndices[currentCursorIndex]);
    if (v !== null && Number.isFinite(v)) return v;
  }
  return fallback;
}

// Numeric labels updates helper
function updateNumericDisplays(row, gpsPositionOverride = null, displayTimeOverride = null) {
  const displayTime = Number.isFinite(displayTimeOverride) ? displayTimeOverride : row.time_sec;
  if (currentTimeVal) {
    let timeText = displayTime.toFixed(2) + 's';
    if (row.gps_time && row.gps_time.trim() !== "" && row.gps_time !== "00:00:00.00") {
      timeText += ` (${row.gps_time})`;
    }
    currentTimeVal.textContent = timeText;
  }

  if (scrollBar && tabGps && tabGps.classList.contains('active')) {
    scrollBar.value = displayTime.toFixed(2);
    if (gpsPlayTime) gpsPlayTime.textContent = `${displayTime.toFixed(2)} s`;
  }

  // Page 1 Labels (노이즈 필터 적용값 기준)
  cursorSpeed.textContent = cursorChannelValue('fl_speed', row.fl_speed_kmh || 0).toFixed(1);
  if (cursorSpeedRl) cursorSpeedRl.textContent = cursorChannelValue('rl_speed', row.rl_speed_kmh || 0).toFixed(1);
  if (cursorSpeedRr) cursorSpeedRr.textContent = cursorChannelValue('rr_speed', row.rr_speed_kmh || 0).toFixed(1);
  cursorRpm.textContent = Math.round(cursorChannelValue('rpm', row.rpm || 0));

  const gearVal = Math.round(cursorChannelValue('gear', row.gear !== undefined ? row.gear : NaN));
  if (gearVal === 0) {
    cursorGear.textContent = 'N';
  } else {
    cursorGear.textContent = Number.isFinite(gearVal) ? gearVal : '-';
  }

  const steeringDeg = cursorChannelValue('steering', getCalibratedSteering(row.steering_raw));
  cursorSteering.textContent = (steeringDeg >= 0 ? '+' : '') + steeringDeg.toFixed(1);

  if (steeringWheelGraphic) {
    steeringWheelGraphic.style.transform = `rotate(${steeringDeg}deg)`;
  }

  const throttleVal = cursorChannelValue('throttle', row.decoded_tps || 0).toFixed(1);
  const brakeVal = cursorChannelValue('brake', getCalibratedBrake(row.front_brake_raw)).toFixed(1);

  cursorThrottle.textContent = throttleVal;
  cursorBrake.textContent = brakeVal;

  // Page 2 Labels
  diagCursorThrottle.textContent = throttleVal;
  diagCursorBrake.textContent = brakeVal;
  diagCursorSteering.textContent = (steeringDeg >= 0 ? '+' : '') + steeringDeg.toFixed(1);

  // 2페이지 핸들 그래픽 회전 연동
  const diagWheel = document.getElementById('diag-steering-wheel-graphic');
  if (diagWheel) {
    diagWheel.style.transform = `rotate(${steeringDeg}deg)`;
  }

  // 3페이지(GPS 지도) 우측 상단 조향각 위젯 연동
  if (gpsSteeringWheelGraphic) {
    gpsSteeringWheelGraphic.style.transform = `rotate(${steeringDeg}deg)`;
  }
  if (gpsCursorSteering) {
    gpsCursorSteering.textContent = (steeringDeg >= 0 ? '+' : '') + steeringDeg.toFixed(1);
  }

  const susText = (key, wheel, raw) => {
    const v = cursorChannelValue(key, getCalibratedSuspension(wheel, raw));
    return Number.isFinite(v) ? `${v.toFixed(2)} mm` : '----';
  };
  cursorSusFl.textContent = susText('sus_fl', 'fl', row.suspension_fl_raw);
  cursorSusFr.textContent = susText('sus_fr', 'fr', row.suspension_fr_raw);
  cursorSusRl.textContent = susText('sus_rl', 'rl', row.suspension_rl_raw);
  cursorSusRr.textContent = susText('sus_rr', 'rr', row.suspension_rr_raw);

  // Page 4 temperature values
  if (tempCursorCoolant) tempCursorCoolant.textContent = Math.round(cursorChannelValue('water', row.water_c || 0));
  if (tempCursorOil) tempCursorOil.textContent = Math.round(cursorChannelValue('oil', row.oil_c || 0));
  if (tempCursorIat) tempCursorIat.textContent = Math.round(cursorChannelValue('iat', row.iat_c || 0));
  if (tempCursorEcu) tempCursorEcu.textContent = Math.round(cursorChannelValue('ecu', row.ecu_c || 0));

  // Page 3 GPS Elements update
  if (cursorGpsCoords) {
    const lat = gpsPositionOverride ? gpsPositionOverride.lat : convertNmeaToDecimal(row.gps_lat, false);
    const lon = gpsPositionOverride ? gpsPositionOverride.lon : convertNmeaToDecimal(row.gps_lon, true);
    if (Number.isFinite(lat) && Number.isFinite(lon)) {
      cursorGpsCoords.textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
      
      // Update cursor marker on the map
      if (gpsMap) {
        if (!gpsCursorMarker) {
          const pulseIcon = L.divIcon({
            className: 'custom-div-icon',
            html: '<div class="gps-position-cursor"></div>',
            iconSize: [12, 12],
            iconAnchor: [6, 6]
          });
          gpsCursorMarker = L.marker([lat, lon], { icon: pulseIcon, zIndexOffset: 10000 }).addTo(gpsMap);
        } else {
          gpsCursorMarker.setLatLng([lat, lon]);
        }
        gpsCursorMarker.setZIndexOffset(10000);
        gpsCursorMarker.getElement()?.classList.add('gps-cursor-top');
        updateGpsCursorScale();
        updateGpsCursorLapColor(displayTime);
      }
    } else {
      cursorGpsCoords.textContent = '--.------, ---.------';
    }
  }

  // GPS 속도 vs FL 휠속도 비교 (휠 슬립 / 속도 보정 오차 확인용)
  const gpsSpd = parseFloat(row.gps_speed_kmh) || 0.0;
  const wheelSpd = cursorChannelValue('fl_speed', row.fl_speed_kmh || 0);
  if (gpsCursorSpeed) gpsCursorSpeed.textContent = gpsSpd.toFixed(1);
  if (gpsFullscreenSpeedValue) gpsFullscreenSpeedValue.textContent = gpsSpd.toFixed(1);
  if (gpsCursorWheelSpeed) gpsCursorWheelSpeed.textContent = wheelSpd.toFixed(1);
  if (gpsSpeedDelta) {
    const d = wheelSpd - gpsSpd;
    gpsSpeedDelta.textContent = (d >= 0 ? '+' : '') + d.toFixed(1) + ' km/h';
    // 저속에서는 GPS 속도 자체가 부정확하므로 판정에서 제외
    const meaningful = gpsSpd > 10 || wheelSpd > 10;
    gpsSpeedDelta.classList.toggle('warn', meaningful && Math.abs(d) > 5);
  }

  if (gpsCursorSats) {
    gpsCursorSats.textContent = row.gps_sat !== undefined ? row.gps_sat : '0';
  }

  if (gpsCursorQual) {
    const qual = parseInt(row.gps_qual) || 0;
    let qualText = 'No Fix';
    if (qual === 1) qualText = 'GPS Fix';
    else if (qual === 2) qualText = 'DGPS';
    else if (qual === 4) qualText = 'RTK Fixed';
    else if (qual === 5) qualText = 'RTK Float';
    gpsCursorQual.textContent = `${qual} (${qualText})`;
  }

  if (gpsCursorTime) {
    gpsCursorTime.textContent = (row.gps_time && row.gps_time.trim() !== "") ? row.gps_time : "00:00:00.00";
  }

  // GPS 페이지 IMU 현황: 지도 커서와 동일한 CSV 행을 사용해 완전히 동기화합니다.
  const ax = Number(row.imu_accel_x_g);
  const ay = Number(row.imu_accel_y_g);
  const roll = Number(row.imu_roll_deg);
  const pitch = Number(row.imu_pitch_deg);
  const yaw = Number(row.imu_yaw_deg);
  const imuValid = [ax, ay].every(Number.isFinite);

  if (imuAccelX) imuAccelX.textContent = Number.isFinite(ax) ? ax.toFixed(2) : '--.--';
  if (imuAccelY) imuAccelY.textContent = Number.isFinite(ay) ? ay.toFixed(2) : '--.--';
  if (imuRoll) imuRoll.textContent = Number.isFinite(roll) ? `${roll.toFixed(1)}°` : '--.-°';
  if (imuPitch) imuPitch.textContent = Number.isFinite(pitch) ? `${pitch.toFixed(1)}°` : '--.-°';
  if (imuYaw) imuYaw.textContent = Number.isFinite(yaw) ? `${yaw.toFixed(1)}°` : '--.-°';

  const batteryPct = Number(row.imu_battery_pct);
  if (imuBattery) imuBattery.textContent = Number.isFinite(batteryPct) ? `${Math.round(batteryPct)}%` : '--%';
  const ageMs = Number(row.imu_age_us) / 1000;
  if (imuAge) {
    imuAge.textContent = Number.isFinite(ageMs) ? `${ageMs.toFixed(0)} ms` : '-- ms';
    imuAge.classList.toggle('stale', Number.isFinite(ageMs) && ageMs > 200);
  }

  if (imuGDot) {
    const limitG = 2.0;
    const clamp = value => Math.max(-limitG, Math.min(limitG, value));
    // Vehicle axes: +X is forward (screen up), +Y is left (screen left).
    const left = imuValid ? 50 - (clamp(ay) / limitG) * 45 : 50;
    const top = imuValid ? 50 - (clamp(ax) / limitG) * 45 : 50;
    imuGDot.style.left = `${left}%`;
    imuGDot.style.top = `${top}%`;
    imuGDot.style.opacity = imuValid ? '1' : '0.25';
  }
}

function handleFile(file) {
  if (!file.name.endsWith('.csv') && !file.name.endsWith('.CSV')) {
    alert('CSV 형식의 로그 파일만 업로드할 수 있습니다.');
    return;
  }

  if (loadedFileBadge) {
    loadedFileBadge.textContent = '📄 ' + file.name;
    loadedFileBadge.style.display = 'inline-block';
  }

  statusBadge.className = 'status-badge active';
  statusText.textContent = '로그 파싱 중...';

  Papa.parse(file, {
    header: true,
    // [중요] CAN 프레임 컬럼은 16진수 문자열이므로 절대 숫자로 변환하면 안 됩니다.
    // dynamicTyping:true 로 전부 변환하면 'E'가 지수 표기로 해석되어
    //   '9E01082950004002' → 9e1082950004002 → Infinity → 'Infinity' → 헥사 정리 → '000000000000000f'
    // 처럼 프레임이 통째로 망가집니다. (Telemetry_001.csv 기준 전체 CAN 프레임의
    // 3.85%인 16,096개가 이 경로로 손상됨 — RPM/기어/온도/서스펜션 값이 순간적으로 튀는 원인)
    // 16자리 전부 숫자인 프레임도 2^53을 넘으면 하위 바이트가 잘려나갑니다.
    dynamicTyping: header => !/^can\d+_data$/i.test(String(header)),
    skipEmptyLines: true,
    complete: function (results) {
      globalData = results.data;
      initDataAndDashboard();
      uploadFileToServer(file);
    },
    error: function (err) {
      statusBadge.className = 'status-badge inactive';
      statusText.textContent = '파싱 오류!';
      alert('CSV 파일을 읽는 중 오류가 발생했습니다: ' + err.message);
    }
  });
}

function uploadFileToServer(file) {
  const formData = new FormData();
  formData.append('csvFile', file);

  fetch('/api/upload', {
    method: 'POST',
    body: formData
  })
  .then(response => {
    if (!response.ok) throw new Error('Upload failed');
    return response.json();
  })
  .then(data => {
    console.log('서버 업로드 성공:', data);
  })
  .catch(err => {
    console.error('서버 업로드 에러:', err);
  });
}

function initDataAndDashboard() {
  if (globalData.length === 0) return;
  setGpsPlayback(false);
  statusText.textContent = '지표 연산 중...';

  globalData = globalData.map(normalizeTelemetryRow);
  const startUs = globalData[0].timestamp_us || 0;
  
  let latestRpm = 0;
  let latestTps = 0.0;
  let latestSpeedKmh = 0.0;
  let latestOilC = 0;
  let latestWaterC = 0;
  let latestGear = 0;
  let latestBatteryMv = 0;
  let latestIatC = 0;
  let latestEcuC = 0;
  let latestEmuAdc4Raw = 0;
  let latestEmuAdc5Raw = 0;
  let latestEmuAdc6Raw = 0;

  globalData.forEach(row => {
    // timestamp_us in the new CSV is already in seconds, so we don't divide by 1,000,000!
    row.time_sec = (row.timestamp_us || 0) - startUs;

    // IMU logger units → dashboard engineering units.
    row.imu_gyro_x_dps = Number(row.imu_gyro_x_deci_dps) / 10.0;
    row.imu_gyro_y_dps = Number(row.imu_gyro_y_deci_dps) / 10.0;
    row.imu_gyro_z_dps = Number(row.imu_gyro_z_deci_dps) / 10.0;
    row.imu_accel_x_g = Number(row.imu_accel_x_milli_g) / 1000.0;
    row.imu_accel_y_g = Number(row.imu_accel_y_milli_g) / 1000.0;
    row.imu_accel_z_g = Number(row.imu_accel_z_milli_g) / 1000.0;
    row.imu_roll_deg = Number(row.imu_roll_centi_deg) / 100.0;
    row.imu_pitch_deg = Number(row.imu_pitch_centi_deg) / 100.0;
    row.imu_yaw_deg = Number(row.imu_yaw_centi_deg) / 100.0;

    const hasPackedCan = row.can600_data !== undefined;

    if (hasPackedCan) {
      const can600 = decodePackedCanFrame(row.can600_data);
      const can601 = decodePackedCanFrame(row.can601_data);
      const can602 = decodePackedCanFrame(row.can602_data);
      const can604 = decodePackedCanFrame(row.can604_data);
      const can606 = decodePackedCanFrame(row.can606_data);

      if (packedCanFrameHasData(can600)) {
        latestRpm = can600[0] | (can600[1] << 8);
        latestTps = can600[2] * 0.5;
        latestIatC = can600[3] > 127 ? can600[3] - 256 : can600[3];
      }
      if (packedCanFrameHasData(can602)) {
        latestSpeedKmh = can602[0] | (can602[1] << 8);

        latestOilC = can602[3];

        let water = can602[6] | (can602[7] << 8);
        if (water > 32767) water -= 65536;
        latestWaterC = water;
      }

      if (packedCanFrameHasData(can604)) {
        latestGear = can604[0];
        latestEcuC = can604[1] > 127 ? can604[1] - 256 : can604[1];
        latestBatteryMv = (can604[2] | (can604[3] << 8)) * 27;
      }
      if (packedCanFrameHasData(can601)) {
        latestEmuAdc4Raw = can601[6] | (can601[7] << 8);
      }
      if (packedCanFrameHasData(can606)) {
        latestEmuAdc5Raw = can606[0] | (can606[1] << 8);
        latestEmuAdc6Raw = can606[2] | (can606[3] << 8);
      }
    } else {
      // Legacy logs store one CAN frame in each CSV row.
      const stdId = parseHexOrInt(row.can_id);
      const dlc = parseHexOrInt(row.can_dlc);
      const canData = [
        parseHexOrInt(row.can_d0),
        parseHexOrInt(row.can_d1),
        parseHexOrInt(row.can_d2),
        parseHexOrInt(row.can_d3),
        parseHexOrInt(row.can_d4),
        parseHexOrInt(row.can_d5),
        parseHexOrInt(row.can_d6),
        parseHexOrInt(row.can_d7)
      ];

      if (row.can_valid === 1 && dlc >= 8) {
        if (stdId === 0x600) {
          latestRpm = canData[0] | (canData[1] << 8);
          latestTps = canData[2] * 0.5;
          latestIatC = canData[3] > 127 ? canData[3] - 256 : canData[3];
        } else if (stdId === 0x601) {
          latestEmuAdc4Raw = canData[6] | (canData[7] << 8);
        } else if (stdId === 0x602) {
          latestSpeedKmh = canData[0] | (canData[1] << 8);
          
          latestOilC = canData[3];

          let water = canData[6] | (canData[7] << 8);
          if (water > 32767) water -= 65536;
          latestWaterC = water;
        } else if (stdId === 0x604) {
          latestGear = canData[0];
          latestEcuC = canData[1] > 127 ? canData[1] - 256 : canData[1];
          latestBatteryMv = (canData[2] | (canData[3] << 8)) * 27;
        } else if (stdId === 0x606) {
          latestEmuAdc5Raw = canData[0] | (canData[1] << 8);
          latestEmuAdc6Raw = canData[2] | (canData[3] << 8);
        }
      }
    }

    // Set parsed values on the row object so the rest of the application uses them naturally
    row.rpm = latestRpm;
    row.decoded_tps = latestTps;
    row.can_speed_kmh = latestSpeedKmh;
    row.oil_c = latestOilC;
    row.water_c = latestWaterC;
    row.iat_c = latestIatC;
    row.ecu_c = latestEcuC;
    row.gear = latestGear;
    row.battery_mV = latestBatteryMv;
    row.suspension_rl_raw = latestEmuAdc4Raw;
    row.suspension_rr_raw = latestEmuAdc5Raw;
    row.rear_brake_raw = latestEmuAdc6Raw;

    // Front-left wheel speed comes from EMU VSS (0x602 bytes 0..1).
    row.fl_speed_kmh = latestSpeedKmh;
    // Rear-left wheel speed comes from the dedicated datalogger wheel channel.
    row.rl_speed_kmh = (parseHexOrInt(row.rl_wheel_speed_centi_kmh ??
      row.wheel4_speed_centi_kmh) || 0) / 100.0;
    // Rear-right wheel speed comes from Wheel Speed 3 on the datalogger.
    row.rr_speed_kmh = (parseHexOrInt(row.rr_wheel_speed_centi_kmh ??
      row.wheel3_speed_centi_kmh) || 0) / 100.0;
  });

  let lastValidRow = globalData[globalData.length - 1];
  for (let i = globalData.length - 1; i >= 0; i--) {
    if (globalData[i] && globalData[i].time_sec !== undefined && !isNaN(globalData[i].time_sec)) {
      lastValidRow = globalData[i];
      break;
    }
  }
  totalDurationSec = lastValidRow.time_sec || 0.1;

  limitStartSec = 0;
  limitEndSec = totalDurationSec;

  let maxRpm = 0;
  let maxSpeed = 0.0;
  let minBattmV = 99999;
  let hasAdcAnomaly = false;
  let maxCoolantC = -Infinity;
  let maxOilC = -Infinity;
  let maxIatC = -Infinity;
  let maxEcuC = -Infinity;

  globalData.forEach(row => {
    if (row.rpm > maxRpm) maxRpm = row.rpm;
    const speed = row.fl_speed_kmh || 0;
    if (speed > maxSpeed) maxSpeed = speed;
    if (row.battery_mV && row.battery_mV > 0 && row.battery_mV < minBattmV) {
      minBattmV = row.battery_mV;
    }
    if (row.suspension_fl_raw > 3800 && row.suspension_rl_raw > 3800) {
      hasAdcAnomaly = true;
    }
    if (Number.isFinite(row.water_c)) maxCoolantC = Math.max(maxCoolantC, row.water_c);
    if (Number.isFinite(row.oil_c)) maxOilC = Math.max(maxOilC, row.oil_c);
    if (Number.isFinite(row.iat_c)) maxIatC = Math.max(maxIatC, row.iat_c);
    if (Number.isFinite(row.ecu_c)) maxEcuC = Math.max(maxEcuC, row.ecu_c);
  });

  statMaxRpm.textContent = Math.round(maxRpm).toLocaleString();
  statMaxSpeed.textContent = maxSpeed.toFixed(1) + ' km/h';
  statMinBatt.textContent = minBattmV === 99999 ? '0.00 V' : (minBattmV / 1000.0).toFixed(2) + ' V';
  statDuration.textContent = totalDurationSec.toFixed(1) + 's';
  if (tempMaxCoolant) tempMaxCoolant.textContent = Number.isFinite(maxCoolantC) ? Math.round(maxCoolantC) : '--';
  if (tempMaxOil) tempMaxOil.textContent = Number.isFinite(maxOilC) ? Math.round(maxOilC) : '--';
  if (tempMaxIat) tempMaxIat.textContent = Number.isFinite(maxIatC) ? Math.round(maxIatC) : '--';
  if (tempMaxEcu) tempMaxEcu.textContent = Number.isFinite(maxEcuC) ? Math.round(maxEcuC) : '--';
  
  if (adcAlertBadge) {
    adcAlertBadge.style.display = hasAdcAnomaly ? 'block' : 'none';
  }

  // [노이즈 필터] 100Hz 원본 전체를 채널별 배열로 만들어 둡니다.
  // 필터는 반드시 다운샘플링 "이전"의 원본에 적용해야 에일리어싱 없이 동작합니다.
  if (typeof buildRawChannels === 'function') {
    statusText.textContent = '채널 구성 중...';
    buildRawChannels(globalData);
  }

  // [성능 초고속 최적화]: 전체 원본 로그 데이터를 최대 4,500 포인트 크기로 1회 정밀 샘플링하여 꽂아둡니다.
  // 이로 인해 휠 확대 시 11개 차트 객체를 완전 파괴(destroy)하고 새로 그리지 않고, X축 범위(scale min/max)만 초고속 갱신하게 됩니다.
  sampleIndices = downsampleIndices(globalData.length, 4500);
  activeSampledData = sampleIndices.map(i => globalData[i]);
  sampleTimes = activeSampledData.map(r => r.time_sec);

  // 최초 1회 전체 차트 생성 기동
  renderMotecCharts(activeSampledData);
  currentCursorIndex = 0;
  if (activeSampledData[0]) updateNumericDisplays(activeSampledData[0]);

  // 초기 줌 레인지 적용
  applyZoomRange(0, totalDurationSec);

  // GPS 지도 초기화 및 주행 경로 그리기
  if (typeof L !== 'undefined') {
    try {
      initGpsMap();
      
      const routeCoords = [];
      globalData.forEach(row => {
        const lat = convertNmeaToDecimal(row.gps_lat, false);
        const lon = convertNmeaToDecimal(row.gps_lon, true);
        if (lat && lon) {
          routeCoords.push([lat, lon]);
        }
      });

      if (routeCoords.length > 0 && gpsRouteLine) {
        gpsRouteLine.setLatLngs(routeCoords);
        // 자동 줌 및 위치 맞춤
        gpsMap.fitBounds(gpsRouteLine.getBounds(), { padding: [20, 20] });
      } else if (gpsRouteLine) {
        gpsRouteLine.setLatLngs([]);
      }
      gpsLapPoints = buildGpsLapPoints(globalData);
      clearGpsLapAnalysis();
      const restoredFixedLines = restoreGpsFixedLines();
      const initialGpsRow = activeSampledData[currentCursorIndex];
      if (initialGpsRow) updateNumericDisplays(initialGpsRow);
      if (gpsLapFixSummary && gpsLapPoints.length && !restoredFixedLines) {
        gpsLapFixSummary.textContent = `${gpsLapPoints.length.toLocaleString()}개 유효 GPS fix 준비됨`;
      }
    } catch (err) {
      console.error("GPS 지도 설정 실패:", err);
    }
  }

  statusBadge.className = 'status-badge active';
  statusText.textContent = '로그 로드 완료';
}

// [초고속 60fps 최적화 개편] 이제 더 이상 차트를 파괴/재생성하지 않고, X축 범위만 변경하여 갱신합니다!
function applyZoomRange(start, end) {
  if (globalData.length === 0 || activeSampledData.length === 0) return;

  let cleanStart = Math.max(0, start);
  let cleanEnd = Math.min(totalDurationSec, end);
  if (cleanStart >= cleanEnd) {
    cleanEnd = Math.min(cleanStart + 5, totalDurationSec);
  }

  currentStartSec = cleanStart;
  currentEndSec = cleanEnd;

  inputStart.value = cleanStart.toFixed(1);
  inputEnd.value = cleanEnd.toFixed(1);

  // GPS 페이지가 활성화된 경우: 시간 스크러버(Scrubber)로 동작하도록 스크롤바 세팅
  if (tabGps && tabGps.classList.contains('active')) {
    scrollBar.min = cleanStart.toFixed(2);
    scrollBar.max = cleanEnd.toFixed(2);
    scrollBar.step = "0.04";
    
    // 현재 커서 시점 탐색하여 설정
    const currentRow = activeSampledData[currentCursorIndex];
    let targetTime = cleanStart;
    if (currentRow && currentRow.time_sec >= cleanStart && currentRow.time_sec <= cleanEnd) {
      targetTime = currentRow.time_sec;
    }
    scrollBar.value = targetTime.toFixed(2);
    scrollBar.disabled = (cleanEnd - cleanStart <= 0.05);
    if (gpsFullscreenTimeline) {
      gpsFullscreenTimeline.min = scrollBar.min;
      gpsFullscreenTimeline.max = scrollBar.max;
      gpsFullscreenTimeline.step = scrollBar.step;
      gpsFullscreenTimeline.value = scrollBar.value;
      gpsFullscreenTimeline.disabled = scrollBar.disabled;
    }
  } else {
    // 차트 페이지의 경우: 차트 좌우 이동(Panning) 스크롤바로 세팅
    const visibleSpan = cleanEnd - cleanStart;
    const maxScroll = totalDurationSec - visibleSpan;

    scrollBar.min = "0";
    scrollBar.max = Math.max(0, maxScroll).toFixed(2);
    scrollBar.value = cleanStart.toFixed(2);
    scrollBar.step = (totalDurationSec / 2000).toFixed(4);
    scrollBar.disabled = maxScroll <= 0.01;
  }

  // 확대된 IMU 구간은 필터 적용본의 원본 해상도로 다시 구성합니다. 그래야
  // 100 Hz 필터 커서와 화면의 필터 곡선이 같은 좌표를 가리킵니다.
  refreshVisibleImuSeries(cleanStart, cleanEnd, false);

  // 11개 Chart.js 인스턴스의 X축 범위만 갱신 (CPU 오버헤드 99% 해제!)
  const targetCharts = [
    chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
    diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
    chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
  ];

  targetCharts.forEach(c => {
    if (c && c.options.scales && c.options.scales.x) {
      c.options.scales.x.min = cleanStart;
      c.options.scales.x.max = cleanEnd;
      c.update('none'); // 애니메이션을 끄고 즉각 60fps 무지연 드로잉!
    }
  });

  if (globalData.length > 0 && activeSampledData.length > 0) {
    drawCssIntersectionDots(currentCursorIndex);
  } else {
    clearAllDomCursors();
  }
}

function filterDataByRange(start, end) {
  return globalData.filter(row => row.time_sec >= start && row.time_sec <= end);
}

function downsampleData(data, limit) {
  if (data.length <= limit) return data;
  const step = Math.floor(data.length / limit);
  const result = [];
  for (let i = 0; i < data.length; i += step) {
    result.push(data[i]);
  }
  if (result[result.length - 1] !== data[data.length - 1]) {
    result.push(data[data.length - 1]);
  }
  return result;
}

// 위 downsampleData와 동일한 규칙으로 "인덱스"만 뽑습니다.
// 필터가 적용된 100Hz 배열에서 화면용 값을 추출할 때 사용합니다.
function downsampleIndices(len, limit) {
  const idx = [];
  if (len <= 0) return idx;
  if (len <= limit) {
    for (let i = 0; i < len; i++) idx.push(i);
    return idx;
  }
  const step = Math.floor(len / limit);
  for (let i = 0; i < len; i += step) idx.push(i);
  if (idx[idx.length - 1] !== len - 1) idx.push(len - 1);
  return idx;
}

// Rebuild only the visible IMU series from the full-resolution filtered
// channels. A single 4,500-point sample of an entire long run can omit a peak;
// after zooming that made the exact filtered cursor appear away from the line.
function refreshVisibleImuSeries(startTime, endTime, updateNow = true) {
  if (!globalData.length || typeof channelSeries !== 'function') return;

  let lo = 0;
  let hi = globalData.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (globalData[mid].time_sec < startTime) lo = mid + 1;
    else hi = mid;
  }
  const first = Math.max(0, lo - 1);

  lo = first;
  hi = globalData.length - 1;
  while (lo < hi) {
    const mid = ((lo + hi + 1) >> 1);
    if (globalData[mid].time_sec > endTime) hi = mid - 1;
    else lo = mid;
  }
  const last = Math.min(globalData.length - 1, lo + 1);
  const count = Math.max(1, last - first + 1);
  const step = Math.max(1, Math.ceil(count / 4500));
  const indices = [];
  for (let i = first; i <= last; i += step) indices.push(i);
  if (indices[indices.length - 1] !== last) indices.push(last);
  const times = indices.map(index => globalData[index].time_sec);

  const charts = [
    [chartImuAccel, ['imu_ax', 'imu_ay']],
    [chartImuGyro, ['imu_gx', 'imu_gy', 'imu_gz']]
  ];
  charts.forEach(([chart, keys]) => {
    if (!chart) return;
    keys.forEach((key, datasetIndex) => {
      if (chart.data.datasets[datasetIndex]) {
        chart.data.datasets[datasetIndex].data = channelSeries(key, indices, times);
      }
    });
    if (updateNow) chart.update('none');
  });
}

// 필터 설정이 바뀌었을 때 모든 차트의 데이터셋을 교체하고 즉시 다시 그립니다.
function refreshChartsAfterFilter() {
  if (!globalData.length || !sampleIndices.length) return;

  const pairs = [
    [chartSpeed, 'chart-ground-speed'],
    [chartRpm, 'chart-engine-rpm'],
    [chartGear, 'chart-vehicle-gear'],
    [chartSteering, 'chart-steering-angle'],
    [chartThrottleBrake, 'chart-throttle-brake'],
    [diagChartThrottleBrake, 'diag-chart-throttle-brake'],
    [diagChartSteering, 'diag-chart-steering'],
    [chartFL, 'chart-sus-fl'],
    [chartFR, 'chart-sus-fr'],
    [chartRL, 'chart-sus-rl'],
    [chartRR, 'chart-sus-rr'],
    [chartCoolantOil, 'chart-coolant-oil'],
    [chartIntakeEcu, 'chart-intake-ecu']
  ];

  pairs.forEach(([chart, canvasId]) => {
    if (!chart) return;
    const keys = CHART_CHANNELS[canvasId];
    if (!keys) return;
    keys.forEach((key, i) => {
      if (chart.data.datasets[i]) {
        chart.data.datasets[i].data = channelSeries(key, sampleIndices, sampleTimes);
      }
    });
    chart.update('none');
  });
  refreshVisibleImuSeries(currentStartSec, currentEndSec);

  if (typeof refreshFilterBadges === 'function') refreshFilterBadges();

  const row = activeSampledData[currentCursorIndex];
  if (row) updateNumericDisplays(row);
  drawCssIntersectionDots(currentCursorIndex);
}

let hoverSyncPending = false;
let lastActiveChart = null;
let lastChartEvent = null;

function syncHover(activeChart, chartEvent) {
  if (isKeyboardNavigating) return;
  lastActiveChart = activeChart;
  lastChartEvent = chartEvent;

  if (hoverSyncPending) return;
  hoverSyncPending = true;

  requestAnimationFrame(() => {
    if (!lastActiveChart || !lastChartEvent) {
      hoverSyncPending = false;
      return;
    }
    const points = lastActiveChart.getElementsAtEventForMode(lastChartEvent, 'index', { intersect: false }, true);
    if (points && points.length) {
      const index = points[0].index;
      currentCursorIndex = index;
      const row = activeSampledData[index];
      if (row) {
        drawCssIntersectionDots(index);
        updateNumericDisplays(row);
      }
    }
    hoverSyncPending = false;
  });
}

// On the GPS + IMU page, hovering must not move the synchronized playback
// cursor. Only an intentional press-and-drag gesture scrubs the exact time.
function bindGpsImuDragCursor(chart) {
  if (!chart || !chart.canvas) return;
  const canvas = chart.canvas;
  if (canvas._gpsImuDragHandlers) {
    const old = canvas._gpsImuDragHandlers;
    canvas.removeEventListener('pointerdown', old.down);
    canvas.removeEventListener('pointermove', old.move);
    canvas.removeEventListener('pointerup', old.up);
    canvas.removeEventListener('pointercancel', old.up);
  }

  const scrub = event => {
    const activeChart = canvas.id === 'chart-imu-accel' ? chartImuAccel : chartImuGyro;
    if (!activeChart || !tabGps || !tabGps.classList.contains('active')) return;
    const position = Chart.helpers.getRelativePosition(event, activeChart);
    const targetTime = activeChart.scales.x.getValueForPixel(position.x);
    if (Number.isFinite(targetTime)) updateGpsCursorAtTime(targetTime);
  };
  const down = event => {
    if (!tabGps || !tabGps.classList.contains('active')) return;
    setGpsPlayback(false);
    gpsImuCursorDragging = true;
    canvas.classList.add('cursor-dragging');
    canvas.setPointerCapture(event.pointerId);
    scrub(event);
  };
  const move = event => {
    if (gpsImuCursorDragging && canvas.hasPointerCapture(event.pointerId)) scrub(event);
  };
  const up = event => {
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    gpsImuCursorDragging = false;
    canvas.classList.remove('cursor-dragging');
  };

  canvas.addEventListener('pointerdown', down);
  canvas.addEventListener('pointermove', move);
  canvas.addEventListener('pointerup', up);
  canvas.addEventListener('pointercancel', up);
  canvas._gpsImuDragHandlers = { down, move, up };
}

// 최초 1회만 데이터셋을 세팅하여 생성하는 팩토리
function renderMotecCharts(data) {
  // Canvas is already in use 에러 방지를 위해 기존 차트 객체들을 파괴(destroy)하고 초기화
  const allCharts = [
    chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
    diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
    chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
  ];
  allCharts.forEach(c => {
    if (c) {
      try {
        c.destroy();
      } catch (err) {
        console.warn("기존 차트 파괴 실패:", err);
      }
    }
  });

  chartSpeed = null;
  chartRpm = null;
  chartGear = null;
  chartSteering = null;
  chartThrottleBrake = null;
  diagChartThrottleBrake = null;
  diagChartSteering = null;
  chartFL = null;
  chartFR = null;
  chartRL = null;
  chartRR = null;
  chartCoolantOil = null;
  chartIntakeEcu = null;
  chartImuAccel = null;
  chartImuGyro = null;

  const labels = data.map(r => r.time_sec);

  // 채널 키로 {x,y} 시리즈를 뽑는 헬퍼. 노이즈 필터가 적용된 값을 사용합니다.
  // (filters.js 미로딩 등 예외 상황에서는 원본 row에서 직접 계산해 폴백)
  const S = (key, fallbackFn) => {
    if (typeof channelSeries === 'function' && sampleIndices.length) {
      return channelSeries(key, sampleIndices, sampleTimes);
    }
    return data.map(r => ({ x: r.time_sec, y: fallbackFn(r) }));
  };

  const isDark = document.body.classList.contains('dark-mode');
  const gridColor = isDark ? 'rgba(255, 255, 255, 0.03)' : 'rgba(0, 0, 0, 0.04)';
  const tickColor = isDark ? '#8c96a8' : '#64748b';

  const getCommonOptions = (forcedMinY = null, forcedMaxY = null, yTicks = {}) => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false, // 줌 성능 향상을 위해 일체의 애니메이션 해제
    parsing: false,   // 성능 병목 방지
    normalized: true, // 고속 인덱싱 최적화
    spanGaps: true,
    interaction: { mode: 'index', intersect: false },
    layout: { padding: { top: 4, bottom: 4, right: 0, left: 0 } },
    plugins: {
      legend: { display: false },
      tooltip: { enabled: false }
    },
    scales: {
      x: {
        type: 'linear', // 시차 줌 물리 변환 연동을 위해 linear 타입으로 셋팅
        min: 0,
        max: totalDurationSec,
        grid: { color: gridColor },
        ticks: { color: tickColor, font: { family: 'JetBrains Mono', size: 9 } },
        afterFit(axis) { axis.paddingRight = 10; }
      },
      y: {
        min: forcedMinY,
        max: forcedMaxY,
        grid: { color: gridColor },
        ticks: { color: tickColor, font: { family: 'JetBrains Mono', size: 9 }, ...yTicks },
        afterFit(axis) { axis.width = 55; }
      }
    },
    onHover: (e, elements) => {
      if (tabGps && tabGps.classList.contains('active') &&
          (e.chart.canvas.id === 'chart-imu-accel' || e.chart.canvas.id === 'chart-imu-gyro')) {
        return;
      }
      if (elements && elements.length > 0) {
        const allCharts = {
          chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
          diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
          chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
        };
        for (const [key, chart] of Object.entries(allCharts)) {
          if (chart && chart.canvas === e.chart.canvas) {
            syncHover(chart, e);
            break;
          }
        }
      }
    }
  });

  // ==================== PAGE 1 CHARTS ====================

  const ctxSpeed = document.getElementById('chart-ground-speed').getContext('2d');
  chartSpeed = new Chart(ctxSpeed, {
    type: 'line',
    data: {
      datasets: [
        {
          label: 'FL Wheel Speed',
          data: S('fl_speed', r => r.fl_speed_kmh || 0),
          borderColor: '#f97316',
          borderWidth: 1.5,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'RL Wheel Speed',
          data: S('rl_speed', r => r.rl_speed_kmh || 0),
          borderColor: '#2563eb',
          borderWidth: 1.4,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'RR Wheel Speed',
          data: S('rr_speed', r => r.rr_speed_kmh || 0),
          borderColor: '#16a34a',
          borderWidth: 1.4,
          pointRadius: 0,
          fill: false
        }
      ]
    },
    options: getCommonOptions(0)
  });

  const ctxRpm = document.getElementById('chart-engine-rpm').getContext('2d');
  chartRpm = new Chart(ctxRpm, {
    type: 'line',
    data: {
      datasets: [{
        data: S('rpm', r => r.rpm),
        borderColor: '#dc2626',
        borderWidth: 1.5,
        pointRadius: 0,
        fill: false
      }]
    },
    options: getCommonOptions(0)
  });

  const ctxGear = document.getElementById('chart-vehicle-gear').getContext('2d');
  chartGear = new Chart(ctxGear, {
    type: 'line',
    data: {
      datasets: [{
        data: S('gear', r => r.gear),
        borderColor: '#2563eb',
        borderWidth: 1.8,
        pointRadius: 0,
        stepped: 'before',
        fill: false
      }]
    },
    options: getCommonOptions(0, 6, {
      stepSize: 1,
      callback: function(value) {
        if (value === 0) return 'N';
        return value;
      }
    })
  });

  const ctxSteering = document.getElementById('chart-steering-angle').getContext('2d');
  const optionsSteering = getCommonOptions(null);
  optionsSteering.scales.y.min = -250;
  optionsSteering.scales.y.max = 250;
  optionsSteering.scales.y.grid = {
    color: (context) => (context.value === 0 ? '#ff2d55' : gridColor),
    lineWidth: 1
  };
  optionsSteering.scales.yYaw = { position: 'right', display: false, grid: { display: false } };
  optionsSteering.scales.yLat = { position: 'right', min: -2.5, max: 2.5, display: false, grid: { display: false } };
  chartSteering = new Chart(ctxSteering, {
    type: 'line',
    data: {
      datasets: [{
        label: 'Steering Angle',
        data: S('steering', r => getCalibratedSteering(r.steering_raw)),
        borderColor: '#db2777',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }, {
        label: 'Yaw Rate',
        data: S('imu_gz', r => Number(r.imu_gyro_z_dps) || 0),
        borderColor: '#22c55e',
        borderWidth: 1.3,
        borderDash: [7, 4],
        pointRadius: 0,
        fill: false,
        yAxisID: 'yYaw'
      }, {
        label: 'Lateral G',
        data: S('imu_ay', r => Number(r.imu_accel_y_g) || 0),
        borderColor: '#2563eb',
        borderWidth: 1.3,
        borderDash: [3, 4],
        pointRadius: 0,
        fill: false,
        yAxisID: 'yLat'
      }]
    },
    options: optionsSteering
  });

  const ctxThrottleBrake = document.getElementById('chart-throttle-brake').getContext('2d');
  chartThrottleBrake = new Chart(ctxThrottleBrake, {
    type: 'line',
    data: {
      datasets: [
        {
          label: 'Throttle',
          data: S('throttle', r => r.decoded_tps || 0),
          borderColor: '#16a34a',
          borderWidth: 1.2,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'Brake',
          data: S('brake', r => getCalibratedBrake(r.front_brake_raw)),
          borderColor: '#dc2626',
          borderWidth: 1.2,
          pointRadius: 0,
          fill: false
        }
      ]
    },
    options: getCommonOptions(0, 100, { stepSize: 20 })
  });

  // ==================== PAGE 2 CHARTS ====================

  const ctxDiagThrottleBrake = document.getElementById('diag-chart-throttle-brake').getContext('2d');
  diagChartThrottleBrake = new Chart(ctxDiagThrottleBrake, {
    type: 'line',
    data: {
      datasets: [
        {
          label: 'Throttle',
          data: S('throttle', r => r.decoded_tps || 0),
          borderColor: '#16a34a',
          borderWidth: 1.2,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'Brake',
          data: S('brake', r => getCalibratedBrake(r.front_brake_raw)),
          borderColor: '#dc2626',
          borderWidth: 1.2,
          pointRadius: 0,
          fill: false
        }
      ]
    },
    options: getCommonOptions(0, 100, { stepSize: 50 })
  });

  const ctxDiagSteering = document.getElementById('diag-chart-steering').getContext('2d');
  const optionsDiagSteering = getCommonOptions(-200, 200, { stepSize: 200 });
  optionsDiagSteering.scales.y.grid = {
    color: (context) => (context.value === 0 ? '#ff2d55' : gridColor),
    lineWidth: 1
  };
  diagChartSteering = new Chart(ctxDiagSteering, {
    type: 'line',
    data: {
      datasets: [{
        data: S('steering', r => getCalibratedSteering(r.steering_raw)),
        borderColor: '#db2777',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }]
    },
    options: optionsDiagSteering
  });

  const ctxSusFl = document.getElementById('chart-sus-fl').getContext('2d');
  chartFL = new Chart(ctxSusFl, {
    type: 'line',
    data: {
      datasets: [{
        data: S('sus_fl', r => getCalibratedSuspension('fl', r.suspension_fl_raw)),
        borderColor: '#db2777',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }]
    },
    options: getCommonOptions(null, null, { stepSize: 10 })
  });

  const ctxSusRl = document.getElementById('chart-sus-rl').getContext('2d');
  chartRL = new Chart(ctxSusRl, {
    type: 'line',
    data: {
      datasets: [{
        data: S('sus_rl', r => getCalibratedSuspension('rl', r.suspension_rl_raw)),
        borderColor: '#06b6d4',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }]
    },
    options: getCommonOptions(null, null, { stepSize: 10 })
  });

  // ==================== PAGE 3 GPS + IMU CHARTS ====================
  const accelCanvas = document.getElementById('chart-imu-accel');
  if (accelCanvas) {
    chartImuAccel = new Chart(accelCanvas.getContext('2d'), {
      type: 'line',
      data: {
        datasets: [
          {
            label: 'Longitudinal G (+Forward X)',
            data: S('imu_ax', r => r.imu_accel_x_g),
            borderColor: '#f97316', borderWidth: 1.1, pointRadius: 0, fill: false
          },
          {
            label: 'Lateral G (+Left Y)',
            data: S('imu_ay', r => r.imu_accel_y_g),
            borderColor: '#2563eb', borderWidth: 1.1, pointRadius: 0, fill: false
          },
        ]
      },
      options: getCommonOptions(-2.5, 2.5, { stepSize: 1 })
    });
  }

  const gyroCanvas = document.getElementById('chart-imu-gyro');
  if (gyroCanvas) {
    chartImuGyro = new Chart(gyroCanvas.getContext('2d'), {
      type: 'line',
      data: {
        datasets: [
          {
            label: 'Roll Rate (X)',
            data: S('imu_gx', r => r.imu_gyro_x_dps),
            borderColor: '#f97316', borderWidth: 1.1, pointRadius: 0, fill: false
          },
          {
            label: 'Pitch Rate (Y)',
            data: S('imu_gy', r => r.imu_gyro_y_dps),
            borderColor: '#2563eb', borderWidth: 1.1, pointRadius: 0, fill: false
          },
          {
            label: 'Yaw Rate (Z)',
            data: S('imu_gz', r => r.imu_gyro_z_dps),
            borderColor: '#16a34a', borderWidth: 1.1, pointRadius: 0, fill: false
          }
        ]
      },
      options: getCommonOptions(-100, 100, { stepSize: 50 })
    });
  }
  applyImuAxisToggleState('chart-imu-accel');
  applyImuAxisToggleState('chart-imu-gyro');
  bindGpsImuDragCursor(chartImuAccel);
  bindGpsImuDragCursor(chartImuGyro);

  // ==================== PAGE 4 TEMPERATURE CHARTS ====================
  const temperatureOptions = getCommonOptions(0, 130, { stepSize: 10 });
  temperatureOptions.scales.y.title = {
    display: true,
    text: 'Temperature [°C]',
    color: tickColor
  };
  temperatureOptions.scales.ySpeed = {
    type: 'linear',
    position: 'right',
    min: 0,
    suggestedMax: 120,
    grid: { drawOnChartArea: false },
    ticks: { color: tickColor, font: { family: 'JetBrains Mono', size: 9 } },
    title: {
      display: true,
      text: 'FL Wheel Speed [km/h]',
      color: tickColor
    }
  };
  temperatureOptions.plugins.legend = {
    display: true,
    position: 'top',
    labels: { color: tickColor, boxWidth: 18, font: { family: 'JetBrains Mono', size: 10 } }
  };
  chartCoolantOil = new Chart(document.getElementById('chart-coolant-oil').getContext('2d'), {
    type: 'line',
    data: {
      datasets: [
        {
          label: 'Coolant',
          data: S('water', r => r.water_c),
          borderColor: '#2563eb',
          borderWidth: 1.6,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'Oil',
          data: S('oil', r => r.oil_c),
          borderColor: '#f97316',
          borderWidth: 1.6,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'FL Wheel Speed',
          data: S('fl_speed', r => r.fl_speed_kmh || 0),
          yAxisID: 'ySpeed',
          borderColor: '#06b6d4',
          borderWidth: 1.4,
          borderDash: [6, 3],
          pointRadius: 0,
          fill: false
        }
      ]
    },
    options: temperatureOptions
  });

  const environmentOptions = getCommonOptions(0, 130, { stepSize: 10 });
  environmentOptions.plugins.legend = {
    display: true,
    position: 'top',
    labels: { color: tickColor, boxWidth: 18, font: { family: 'JetBrains Mono', size: 10 } }
  };
  chartIntakeEcu = new Chart(document.getElementById('chart-intake-ecu').getContext('2d'), {
    type: 'line',
    data: {
      datasets: [
        {
          label: 'Intake Air',
          data: S('iat', r => r.iat_c),
          borderColor: '#16a34a',
          borderWidth: 1.6,
          pointRadius: 0,
          fill: false
        },
        {
          label: 'ECU',
          data: S('ecu', r => r.ecu_c),
          borderColor: '#db2777',
          borderWidth: 1.6,
          pointRadius: 0,
          fill: false
        }
      ]
    },
    options: environmentOptions
  });

  const ctxSusFr = document.getElementById('chart-sus-fr').getContext('2d');
  chartFR = new Chart(ctxSusFr, {
    type: 'line',
    data: {
      datasets: [{
        data: S('sus_fr', r => getCalibratedSuspension('fr', r.suspension_fr_raw)),
        borderColor: '#dc2626',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }]
    },
    options: getCommonOptions(null, null, { stepSize: 10 })
  });

  const ctxSusRr = document.getElementById('chart-sus-rr').getContext('2d');
  chartRR = new Chart(ctxSusRr, {
    type: 'line',
    data: {
      datasets: [{
        data: S('sus_rr', r => getCalibratedSuspension('rr', r.suspension_rr_raw)),
        borderColor: '#2563eb',
        borderWidth: 1.2,
        pointRadius: 0,
        fill: false
      }]
    },
    options: getCommonOptions(null, null, { stepSize: 10 })
  });

  // 테마 상태에 맞는 차트 선 색상(다크모드 전용 파스텔톤 포함) 즉시 동기화
  updateChartsTheme();

  // 라벨바에 현재 적용된 노이즈 필터 배지 표시
  if (typeof refreshFilterBadges === 'function') refreshFilterBadges();

  // 조향 보정값에 맞춰 조향 차트 Y축 범위 조정 및 핸들 위젯 상태 표시
  if (typeof updateSteeringAxisRange === 'function') updateSteeringAxisRange();
  if (typeof updateSteeringCalBadges === 'function') updateSteeringCalBadges();
}

// 그래프 우클릭 → 노이즈 필터 메뉴 활성화
if (typeof initFilterContextMenu === 'function') {
  initFilterContextMenu();
}

// 핸들 그래픽 클릭 → 조향 영점 보정 패널 활성화
if (typeof initSteeringCalibration === 'function') {
  initSteeringCalibration();
}
if (typeof initSuspensionCalibration === 'function') {
  initSuspensionCalibration();
}

// 5번 탭: 실시간 무선 텔레메트리 초기화
if (typeof rtInit === 'function') {
  rtInit();
}

let arrowRepeatCount = 0;
let isKeyboardNavigating = false;
let keyboardNavTimer = null;

window.addEventListener('keyup', (e) => {
  if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
    arrowRepeatCount = 0;
  }
});

// ==================== [키보드 단축키 지원 (Keyboard Shortcuts)] ====================
window.addEventListener('keydown', (e) => {
  // 입력 필드에 포커스가 있을 때는 단축키를 비활성화합니다.
  if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'TEXTAREA' || document.activeElement.tagName === 'SELECT') {
    return;
  }

  if (globalData.length === 0 || activeSampledData.length === 0) return;

  const key = e.key;

  // 키보드로 조작하는 순간 마우스 호버로 인한 오버라이드를 차단하고 네이티브 호버 서클을 제거합니다.
  if (key === 'ArrowLeft' || key === 'ArrowRight' || key === 'ArrowUp' || key === 'ArrowDown' || key === ' ' || key === 'Spacebar') {
    isKeyboardNavigating = true;
    clearTimeout(keyboardNavTimer);
    keyboardNavTimer = setTimeout(() => {
      isKeyboardNavigating = false;
    }, 800); // 키보드 입력 중단 후 800ms 동안 마우스 반응 차단

    // 모든 차트에서 기존 마우스 위치에 남아있는 네이티브 active/hover 서클 제거
    const targetCharts = [
      chartSpeed, chartRpm, chartGear, chartSteering, chartThrottleBrake,
      diagChartThrottleBrake, diagChartSteering, chartFL, chartFR, chartRL, chartRR,
      chartCoolantOil, chartIntakeEcu, chartImuAccel, chartImuGyro
    ];
    targetCharts.forEach(c => {
      if (c) {
        c.setActiveElements([]);
        c.update('none');
      }
    });
  }

  // 1, 2: 탭 전환
  if (key === '1') {
    e.preventDefault();
    switchTab('general');
  } else if (key === '2') {
    e.preventDefault();
    switchTab('diag');
  }

  // GPS + IMU page: Space toggles playback. Other pages retain the existing
  // full-view reset shortcut.
  else if (key === ' ' || key === 'Spacebar') {
    e.preventDefault();
    if (tabGps && tabGps.classList.contains('active')) {
      if (!e.repeat) setGpsPlayback(!gpsPlaybackActive);
    } else {
      applyZoomRange(0, totalDurationSec);
    }
  }

  // Left/Right Arrow: 커서 미세 이동 (지속 입력 시 가속도 적용 및 뷰포트 자동 스크롤)
  else if (key === 'ArrowLeft') {
    e.preventDefault();
    let step = 1;
    if (e.repeat) {
      arrowRepeatCount++;
      step = Math.min(60, Math.floor(1 + (arrowRepeatCount * arrowRepeatCount) / 30));
    } else {
      arrowRepeatCount = 0;
    }
    currentCursorIndex = Math.max(0, currentCursorIndex - step);
    const row = activeSampledData[currentCursorIndex];
    if (row) {
      const targetTime = row.time_sec;
      const currentSpan = currentEndSec - currentStartSec;
      if (targetTime < currentStartSec) {
        let newStart = targetTime;
        let newEnd = targetTime + currentSpan;
        if (newEnd > totalDurationSec) {
          newEnd = totalDurationSec;
          newStart = Math.max(0, totalDurationSec - currentSpan);
        }
        applyZoomRange(newStart, newEnd);
      }
      drawCssIntersectionDots(currentCursorIndex);
      updateNumericDisplays(row);
    }
  } else if (key === 'ArrowRight') {
    e.preventDefault();
    let step = 1;
    if (e.repeat) {
      arrowRepeatCount++;
      step = Math.min(60, Math.floor(1 + (arrowRepeatCount * arrowRepeatCount) / 30));
    } else {
      arrowRepeatCount = 0;
    }
    currentCursorIndex = Math.min(activeSampledData.length - 1, currentCursorIndex + step);
    const row = activeSampledData[currentCursorIndex];
    if (row) {
      const targetTime = row.time_sec;
      const currentSpan = currentEndSec - currentStartSec;
      if (targetTime > currentEndSec) {
        let newEnd = targetTime;
        let newStart = targetTime - currentSpan;
        if (newStart < 0) {
          newStart = 0;
          newEnd = Math.min(currentSpan, totalDurationSec);
        }
        applyZoomRange(newStart, newEnd);
      }
      drawCssIntersectionDots(currentCursorIndex);
      updateNumericDisplays(row);
    }
  }

  // Up/Down Arrow / I/O: 확대/축소 (현재 활성 커서 시간 기준)
  else if (key === 'ArrowUp' || key.toLowerCase() === 'i') {
    e.preventDefault();
    const currentSpan = currentEndSec - currentStartSec;
    const targetTime = activeSampledData[currentCursorIndex] ? activeSampledData[currentCursorIndex].time_sec : (currentStartSec + currentEndSec) / 2;
    const newSpan = Math.max(2.0, currentSpan * 0.85); // 15% 줌인
    const ratio = currentSpan > 0 ? (targetTime - currentStartSec) / currentSpan : 0.5;
    let newStart = targetTime - (newSpan * ratio);
    let newEnd = targetTime + (newSpan * (1 - ratio));

    if (newStart < 0) {
      newStart = 0;
      newEnd = Math.min(newSpan, totalDurationSec);
    }
    if (newEnd > totalDurationSec) {
      newEnd = totalDurationSec;
      newStart = Math.max(0, totalDurationSec - newSpan);
    }
    applyZoomRange(newStart, newEnd);
  } else if (key === 'ArrowDown' || key.toLowerCase() === 'o') {
    e.preventDefault();
    const currentSpan = currentEndSec - currentStartSec;
    const targetTime = activeSampledData[currentCursorIndex] ? activeSampledData[currentCursorIndex].time_sec : (currentStartSec + currentEndSec) / 2;
    const newSpan = Math.min(totalDurationSec, currentSpan * 1.15); // 15% 줌아웃
    const ratio = currentSpan > 0 ? (targetTime - currentStartSec) / currentSpan : 0.5;
    let newStart = targetTime - (newSpan * ratio);
    let newEnd = targetTime + (newSpan * (1 - ratio));

    if (newStart < 0) {
      newStart = 0;
      newEnd = Math.min(newSpan, totalDurationSec);
    }
    if (newEnd > totalDurationSec) {
      newEnd = totalDurationSec;
      newStart = Math.max(0, totalDurationSec - newSpan);
    }
    applyZoomRange(newStart, newEnd);
  }
});
