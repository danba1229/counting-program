const state = {
  model: null,
  videoFile: null,
  objectUrl: null,
  processing: false,
  previewPrimed: false,
  roi: null,
  drawing: false,
  draftRoi: null,
  dragStart: null,
  detections: [],
  tracks: [],
  nextTrackId: 1,
  hourlyCounts: [],
  events: [],
  frameSize: { width: 0, height: 0 },
};

const els = {
  videoInput: document.getElementById("videoInput"),
  videoPreview: document.getElementById("videoPreview"),
  overlayCanvas: document.getElementById("overlayCanvas"),
  viewerStage: document.querySelector(".viewer-stage"),
  emptyState: document.getElementById("emptyState"),
  loadModelButton: document.getElementById("loadModelButton"),
  processButton: document.getElementById("processButton"),
  resetButton: document.getElementById("resetButton"),
  downloadCsvButton: document.getElementById("downloadCsvButton"),
  sampleInterval: document.getElementById("sampleInterval"),
  scoreThreshold: document.getElementById("scoreThreshold"),
  matchDistance: document.getElementById("matchDistance"),
  recountCooldown: document.getElementById("recountCooldown"),
  statusText: document.getElementById("statusText"),
  progressText: document.getElementById("progressText"),
  roiText: document.getElementById("roiText"),
  totalCount: document.getElementById("totalCount"),
  durationLabel: document.getElementById("durationLabel"),
  roiSizeLabel: document.getElementById("roiSizeLabel"),
  resultsBody: document.getElementById("resultsBody"),
  eventList: document.getElementById("eventList"),
};

const overlayCtx = els.overlayCanvas.getContext("2d");

initialize();

function initialize() {
  bindEvents();
  drawOverlay();
}

function bindEvents() {
  els.videoInput.addEventListener("change", handleVideoSelection);
  els.loadModelButton.addEventListener("click", loadModel);
  els.processButton.addEventListener("click", processVideo);
  els.resetButton.addEventListener("click", resetAll);
  els.downloadCsvButton.addEventListener("click", downloadCsv);
  els.videoPreview.addEventListener("loadedmetadata", handleVideoReady);
  els.videoPreview.addEventListener("loadeddata", handlePreviewAvailable);
  els.videoPreview.addEventListener("canplay", handlePreviewAvailable);
  els.videoPreview.addEventListener("error", handleVideoError);
  els.videoPreview.addEventListener("seeked", () => {
    syncCanvasToVideo();
    drawCurrentFrame();
  });
  window.addEventListener("resize", () => {
    syncCanvasToVideo();
    drawCurrentFrame();
  });
  els.overlayCanvas.addEventListener("pointerdown", startRoiDraw);
  els.overlayCanvas.addEventListener("pointermove", updateRoiDraw);
  els.overlayCanvas.addEventListener("pointerup", finishRoiDraw);
  els.overlayCanvas.addEventListener("pointerleave", finishRoiDraw);
  els.overlayCanvas.addEventListener("pointercancel", finishRoiDraw);
}

async function handleVideoSelection(event) {
  const [file] = event.target.files ?? [];
  if (!file) {
    return;
  }

  cleanupObjectUrl();
  resetResults();

  state.videoFile = file;
  state.objectUrl = URL.createObjectURL(file);
  state.previewPrimed = false;
  state.roi = null;
  state.draftRoi = null;
  state.detections = [];

  els.videoPreview.src = state.objectUrl;
  els.videoPreview.load();
  els.viewerStage.classList.remove("has-video");
  els.emptyState.style.display = "grid";
  els.emptyState.textContent = "영상 미리보기를 준비하는 중입니다.";
  setStatus("영상을 불러오는 중입니다.");
  els.processButton.disabled = true;
}

async function handleVideoReady() {
  state.frameSize = {
    width: els.videoPreview.videoWidth,
    height: els.videoPreview.videoHeight,
  };
  els.durationLabel.textContent = formatDuration(els.videoPreview.duration);
  els.viewerStage.classList.add("has-video");
  setStatus("첫 프레임이 준비되었습니다. 관심 구역을 드래그해 지정해 주세요.");
  await primePreviewFrame();
  syncCanvasToVideo();
  updateProcessAvailability();
}

function handlePreviewAvailable() {
  els.emptyState.style.display = "none";
  syncCanvasToVideo();
  drawCurrentFrame();
}

function handleVideoError() {
  els.emptyState.style.display = "grid";
  els.emptyState.textContent = "이 브라우저에서 이 영상 코덱을 표시하지 못했습니다.";
  setStatus("영상 미리보기를 불러오지 못했습니다. 다른 형식의 영상인지 확인해 주세요.");
}

async function loadModel() {
  if (state.model) {
    setStatus("모델이 이미 준비되어 있습니다.");
    return;
  }

  try {
    toggleBusy(true);
    setStatus("사람 감지 모델을 불러오는 중입니다. 처음 한 번은 시간이 걸릴 수 있습니다.");
    state.model = await cocoSsd.load({ base: "mobilenet_v2" });
    setStatus("모델 준비가 끝났습니다.");
  } catch (error) {
    console.error(error);
    setStatus("모델을 불러오지 못했습니다. 인터넷 연결을 확인해 주세요.");
  } finally {
    toggleBusy(false);
    updateProcessAvailability();
  }
}

function syncCanvasToVideo() {
  const rect = els.videoPreview.getBoundingClientRect();
  if (!rect.width || !rect.height) {
    return;
  }

  els.overlayCanvas.width = rect.width;
  els.overlayCanvas.height = rect.height;
  els.overlayCanvas.style.width = `${rect.width}px`;
  els.overlayCanvas.style.height = `${rect.height}px`;
}

async function primePreviewFrame() {
  if (state.previewPrimed || !state.videoFile) {
    return;
  }

  state.previewPrimed = true;

  try {
    if (els.videoPreview.readyState < HTMLMediaElement.HAVE_CURRENT_DATA) {
      await waitForMediaEvent(els.videoPreview, "loadeddata", 2000);
    }
  } catch (_) {
    // Some browsers decode the first frame only after a play attempt.
  }

  try {
    const playPromise = els.videoPreview.play();
    if (playPromise) {
      await Promise.race([playPromise, waitMs(500)]);
    }
    await waitMs(120);
    els.videoPreview.pause();
  } catch (_) {
    // Autoplay may be blocked despite muted playback; seeking still gives us a fallback.
  }

  try {
    if (Math.abs(els.videoPreview.currentTime) > 0.001) {
      await seekVideo(0);
    } else {
      els.videoPreview.currentTime = 0;
    }
  } catch (_) {
    els.videoPreview.currentTime = 0;
  }

  handlePreviewAvailable();
}

function drawCurrentFrame() {
  const width = els.overlayCanvas.width;
  const height = els.overlayCanvas.height;
  if (!width || !height) {
    return;
  }

  overlayCtx.clearRect(0, 0, width, height);
  drawOverlay();
}

function drawOverlay() {
  const width = els.overlayCanvas.width;
  const height = els.overlayCanvas.height;
  if (!width || !height) {
    return;
  }

  overlayCtx.save();
  overlayCtx.lineWidth = 2;

  const roi = state.draftRoi ?? state.roi;
  if (roi) {
    const canvasRoi = frameRectToCanvasRect(roi);
    overlayCtx.fillStyle = "rgba(15, 118, 110, 0.18)";
    overlayCtx.strokeStyle = "#1dc8ba";
    overlayCtx.fillRect(canvasRoi.x, canvasRoi.y, canvasRoi.width, canvasRoi.height);
    overlayCtx.strokeRect(canvasRoi.x, canvasRoi.y, canvasRoi.width, canvasRoi.height);
  }

  state.detections.forEach((detection) => {
    const box = frameRectToCanvasRect({
      x: detection.bbox[0],
      y: detection.bbox[1],
      width: detection.bbox[2],
      height: detection.bbox[3],
    });

    overlayCtx.strokeStyle = detection.inside ? "#ffd166" : "#ffffff";
    overlayCtx.lineWidth = detection.trackId ? 2.5 : 1.5;
    overlayCtx.strokeRect(box.x, box.y, box.width, box.height);
    overlayCtx.fillStyle = "rgba(0, 0, 0, 0.62)";
    overlayCtx.fillRect(box.x, Math.max(0, box.y - 24), 88, 22);
    overlayCtx.fillStyle = "#ffffff";
    overlayCtx.font = "12px 'IBM Plex Sans KR'";
    overlayCtx.fillText(
      `${detection.trackId ?? "?"} / ${(detection.score * 100).toFixed(0)}%`,
      box.x + 6,
      Math.max(15, box.y - 9),
    );
  });

  overlayCtx.restore();
}

function startRoiDraw(event) {
  if (!state.frameSize.width || state.processing) {
    return;
  }

  event.preventDefault();
  els.overlayCanvas.setPointerCapture(event.pointerId);
  const point = getCanvasPoint(event);
  state.dragStart = point;
  state.drawing = true;
  state.draftRoi = canvasRectToFrameRect({
    x: point.x,
    y: point.y,
    width: 0,
    height: 0,
  });
  drawCurrentFrame();
}

function updateRoiDraw(event) {
  if (!state.drawing || !state.dragStart) {
    return;
  }

  event.preventDefault();
  const point = getCanvasPoint(event);
  const rect = normalizeRect({
    x: state.dragStart.x,
    y: state.dragStart.y,
    width: point.x - state.dragStart.x,
    height: point.y - state.dragStart.y,
  });
  state.draftRoi = canvasRectToFrameRect(rect);
  drawCurrentFrame();
}

function finishRoiDraw(event) {
  if (!state.drawing) {
    return;
  }

  if (event) {
    event.preventDefault();
    if (typeof event.clientX === "number" && typeof event.clientY === "number") {
      const point = getCanvasPoint(event);
      const rect = normalizeRect({
        x: state.dragStart.x,
        y: state.dragStart.y,
        width: point.x - state.dragStart.x,
        height: point.y - state.dragStart.y,
      });
      state.draftRoi = canvasRectToFrameRect(rect);
    }

    if (els.overlayCanvas.hasPointerCapture?.(event.pointerId)) {
      els.overlayCanvas.releasePointerCapture(event.pointerId);
    }
  }

  state.drawing = false;
  state.dragStart = null;

  if (state.draftRoi && state.draftRoi.width > 20 && state.draftRoi.height > 20) {
    state.roi = clampFrameRect(state.draftRoi);
    updateRoiLabels();
    setStatus("관심 구역이 설정되었습니다. 분석을 시작할 수 있습니다.");
  } else {
    setStatus("관심 구역이 너무 작습니다. 조금 더 크게 드래그해 주세요.");
  }

  state.draftRoi = null;
  drawCurrentFrame();
  updateProcessAvailability();
}

function getCanvasPoint(event) {
  const rect = els.overlayCanvas.getBoundingClientRect();
  return {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top,
  };
}

async function processVideo() {
  if (state.processing || !state.videoFile || !state.roi) {
    return;
  }

  if (!state.model) {
    await loadModel();
    if (!state.model) {
      return;
    }
  }

  const sampleInterval = Math.max(0.2, Number(els.sampleInterval.value) || 1);
  const scoreThreshold = Math.min(0.95, Math.max(0.1, Number(els.scoreThreshold.value) || 0.55));
  const matchDistance = Math.max(20, Number(els.matchDistance.value) || 90);
  const recountCooldown = Math.max(1, Number(els.recountCooldown.value) || 8);

  resetResults();
  state.processing = true;
  state.tracks = [];
  state.nextTrackId = 1;
  state.detections = [];

  toggleBusy(true);
  setStatus("영상 분석을 시작합니다.");

  try {
    els.videoPreview.pause();

    const duration = els.videoPreview.duration || 0;
    const bins = createHourlyBins(duration);
    const events = [];
    const frameCount = Math.max(1, Math.ceil(duration / sampleInterval));

    for (let index = 0; index < frameCount; index += 1) {
      const currentTime = Math.min(duration, index * sampleInterval);
      await seekVideo(currentTime);

      const predictions = await state.model.detect(els.videoPreview);
      const detections = predictions
        .filter((prediction) => prediction.class === "person" && prediction.score >= scoreThreshold)
        .map((prediction) => {
          const centroid = {
            x: prediction.bbox[0] + prediction.bbox[2] / 2,
            y: prediction.bbox[1] + prediction.bbox[3] / 2,
          };
          return {
            ...prediction,
            centroid,
            inside: pointInRect(centroid, state.roi),
          };
        });

      const frameEvents = updateTracks(
        detections,
        currentTime,
        matchDistance,
        recountCooldown,
        index === 0,
      );
      frameEvents.forEach((frameEvent) => {
        events.push(frameEvent);
        const binIndex = Math.floor(frameEvent.time / 3600);
        if (bins[binIndex]) {
          bins[binIndex].count += 1;
        }
      });

      state.detections = detections;
      renderResults(bins, events);
      drawCurrentFrame();
      updateProgress(Math.round(((index + 1) / frameCount) * 100));
      setStatus(`분석 중... ${index + 1}/${frameCount} 프레임 샘플 처리`);
      await nextFrame();
    }

    state.hourlyCounts = bins;
    state.events = events;
    renderResults(bins, events);
    updateProgress(100);
    setStatus("분석이 완료되었습니다.");
    els.downloadCsvButton.disabled = false;
  } catch (error) {
    console.error(error);
    setStatus("분석 중 오류가 발생했습니다. 파일 형식이나 브라우저 콘솔을 확인해 주세요.");
  } finally {
    state.processing = false;
    toggleBusy(false);
    updateProcessAvailability();
  }
}

function updateTracks(detections, currentTime, matchDistance, recountCooldown, isInitializationFrame) {
  const events = [];
  const usedTrackIds = new Set();

  state.tracks = state.tracks.filter((track) => currentTime - track.lastSeen <= Math.max(3, recountCooldown));

  detections.forEach((detection) => {
    let bestTrack = null;
    let bestDistance = Number.POSITIVE_INFINITY;

    state.tracks.forEach((track) => {
      if (usedTrackIds.has(track.id)) {
        return;
      }
      const distance = getDistance(track.centroid, detection.centroid);
      if (distance < bestDistance && distance <= matchDistance) {
        bestDistance = distance;
        bestTrack = track;
      }
    });

    if (!bestTrack) {
      bestTrack = {
        id: state.nextTrackId,
        centroid: detection.centroid,
        lastSeen: currentTime,
        lastCountTime: Number.NEGATIVE_INFINITY,
        inside: isInitializationFrame ? detection.inside : false,
      };
      state.nextTrackId += 1;
      state.tracks.push(bestTrack);
    } else {
      bestTrack.centroid = detection.centroid;
      bestTrack.lastSeen = currentTime;
    }

    if (detection.inside && !bestTrack.inside && currentTime - bestTrack.lastCountTime >= recountCooldown) {
      bestTrack.lastCountTime = currentTime;
      events.push({
        trackId: bestTrack.id,
        time: currentTime,
      });
    }

    bestTrack.inside = detection.inside;
    detection.trackId = bestTrack.id;
    usedTrackIds.add(bestTrack.id);
  });

  return events;
}

function createHourlyBins(duration) {
  const hours = Math.max(1, Math.ceil(duration / 3600));
  return Array.from({ length: hours }, (_, index) => {
    const start = index * 3600;
    const end = Math.min(duration, (index + 1) * 3600);
    return { start, end, count: 0 };
  });
}

function renderResults(bins, events) {
  const totalCount = bins.reduce((sum, bin) => sum + bin.count, 0);
  els.totalCount.textContent = String(totalCount);

  if (!bins.length) {
    els.resultsBody.innerHTML = `
      <tr>
        <td colspan="2" class="placeholder-cell">아직 분석 결과가 없습니다.</td>
      </tr>
    `;
  } else {
    els.resultsBody.innerHTML = bins
      .map(
        (bin) => `
          <tr>
            <td>${formatHourRange(bin.start, bin.end)}</td>
            <td>${bin.count}</td>
          </tr>
        `,
      )
      .join("");
  }

  if (!events.length) {
    els.eventList.innerHTML = '<li class="placeholder-cell">아직 이벤트가 없습니다.</li>';
  } else {
    els.eventList.innerHTML = events
      .slice()
      .reverse()
      .map((event) => `<li>사람 ${event.trackId} 진입: <strong>${formatTime(event.time)}</strong></li>`)
      .join("");
  }
}

function updateRoiLabels() {
  if (!state.roi) {
    els.roiText.textContent = "관심 구역이 아직 지정되지 않았습니다.";
    els.roiSizeLabel.textContent = "미지정";
    return;
  }

  const { x, y, width, height } = state.roi;
  els.roiText.textContent = `선택된 구역: x=${Math.round(x)}, y=${Math.round(y)}, w=${Math.round(width)}, h=${Math.round(height)}`;
  els.roiSizeLabel.textContent = `${Math.round(width)} x ${Math.round(height)} px`;
}

function updateProcessAvailability() {
  els.processButton.disabled = !state.videoFile || !state.roi || state.processing;
}

function toggleBusy(isBusy) {
  els.loadModelButton.disabled = isBusy;
  els.resetButton.disabled = isBusy;
  els.videoInput.disabled = isBusy;
  els.processButton.disabled = isBusy || !state.videoFile || !state.roi;
}

function resetResults() {
  state.hourlyCounts = [];
  state.events = [];
  state.tracks = [];
  state.nextTrackId = 1;
  state.detections = [];
  els.downloadCsvButton.disabled = true;
  els.totalCount.textContent = "0";
  renderResults([], []);
  updateProgress(0);
}

function resetAll() {
  cleanupObjectUrl();
  state.videoFile = null;
  state.previewPrimed = false;
  state.roi = null;
  state.draftRoi = null;
  state.processing = false;
  state.detections = [];
  state.tracks = [];
  state.nextTrackId = 1;
  state.frameSize = { width: 0, height: 0 };

  els.videoInput.value = "";
  els.videoPreview.removeAttribute("src");
  els.videoPreview.load();
  els.emptyState.style.display = "grid";
  els.emptyState.textContent = "영상 파일을 올리면 여기에 첫 프레임이 표시됩니다.";
  els.viewerStage.classList.remove("has-video");
  els.durationLabel.textContent = "0분";
  updateRoiLabels();
  resetResults();
  setStatus("영상 파일을 선택해 주세요.");
  overlayCtx.clearRect(0, 0, els.overlayCanvas.width, els.overlayCanvas.height);
  updateProcessAvailability();
}

function cleanupObjectUrl() {
  if (state.objectUrl) {
    URL.revokeObjectURL(state.objectUrl);
    state.objectUrl = null;
  }
}

function waitMs(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function waitForMediaEvent(element, eventName, timeoutMs) {
  return new Promise((resolve, reject) => {
    const onEvent = () => {
      cleanup();
      resolve();
    };
    const onError = () => {
      cleanup();
      reject(new Error(`${eventName} 실패`));
    };
    const onTimeout = () => {
      cleanup();
      reject(new Error(`${eventName} timeout`));
    };
    const cleanup = () => {
      clearTimeout(timeoutId);
      element.removeEventListener(eventName, onEvent);
      element.removeEventListener("error", onError);
    };

    const timeoutId = window.setTimeout(onTimeout, timeoutMs);
    element.addEventListener(eventName, onEvent, { once: true });
    element.addEventListener("error", onError, { once: true });
  });
}

async function seekVideo(time) {
  if (Math.abs(els.videoPreview.currentTime - time) < 0.001 && els.videoPreview.readyState >= 2) {
    return;
  }

  await new Promise((resolve, reject) => {
    const onSeeked = () => {
      cleanup();
      resolve();
    };
    const onError = () => {
      cleanup();
      reject(new Error("영상 탐색 실패"));
    };
    const cleanup = () => {
      els.videoPreview.removeEventListener("seeked", onSeeked);
      els.videoPreview.removeEventListener("error", onError);
    };

    els.videoPreview.addEventListener("seeked", onSeeked, { once: true });
    els.videoPreview.addEventListener("error", onError, { once: true });
    els.videoPreview.currentTime = time;
  });
}

function canvasRectToFrameRect(rect) {
  const scaleX = state.frameSize.width / els.overlayCanvas.width;
  const scaleY = state.frameSize.height / els.overlayCanvas.height;
  return {
    x: rect.x * scaleX,
    y: rect.y * scaleY,
    width: rect.width * scaleX,
    height: rect.height * scaleY,
  };
}

function frameRectToCanvasRect(rect) {
  const scaleX = els.overlayCanvas.width / state.frameSize.width;
  const scaleY = els.overlayCanvas.height / state.frameSize.height;
  return {
    x: rect.x * scaleX,
    y: rect.y * scaleY,
    width: rect.width * scaleX,
    height: rect.height * scaleY,
  };
}

function clampFrameRect(rect) {
  const normalized = normalizeRect(rect);
  const x = Math.max(0, Math.min(state.frameSize.width, normalized.x));
  const y = Math.max(0, Math.min(state.frameSize.height, normalized.y));
  const width = Math.max(0, Math.min(state.frameSize.width - x, normalized.width));
  const height = Math.max(0, Math.min(state.frameSize.height - y, normalized.height));
  return { x, y, width, height };
}

function normalizeRect(rect) {
  const x = rect.width < 0 ? rect.x + rect.width : rect.x;
  const y = rect.height < 0 ? rect.y + rect.height : rect.y;
  return {
    x,
    y,
    width: Math.abs(rect.width),
    height: Math.abs(rect.height),
  };
}

function pointInRect(point, rect) {
  return (
    point.x >= rect.x &&
    point.x <= rect.x + rect.width &&
    point.y >= rect.y &&
    point.y <= rect.y + rect.height
  );
}

function getDistance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function updateProgress(percent) {
  els.progressText.textContent = `${percent}%`;
}

function setStatus(message) {
  els.statusText.textContent = message;
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds <= 0) {
    return "0분";
  }
  const minutes = Math.round(seconds / 60);
  if (minutes < 60) {
    return `${minutes}분`;
  }
  const hours = Math.floor(minutes / 60);
  const remaining = minutes % 60;
  return remaining ? `${hours}시간 ${remaining}분` : `${hours}시간`;
}

function formatTime(seconds) {
  const whole = Math.max(0, Math.floor(seconds));
  const hours = String(Math.floor(whole / 3600)).padStart(2, "0");
  const minutes = String(Math.floor((whole % 3600) / 60)).padStart(2, "0");
  const secs = String(whole % 60).padStart(2, "0");
  return `${hours}:${minutes}:${secs}`;
}

function formatHourRange(start, end) {
  return `${formatTime(start)} ~ ${formatTime(end)}`;
}

function nextFrame() {
  return new Promise((resolve) => requestAnimationFrame(() => resolve()));
}

function downloadCsv() {
  if (!state.hourlyCounts.length) {
    return;
  }

  const lines = ["start,end,count"];
  state.hourlyCounts.forEach((bin) => {
    lines.push(`${formatTime(bin.start)},${formatTime(bin.end)},${bin.count}`);
  });

  const blob = new Blob([lines.join("\n")], { type: "text/csv;charset=utf-8;" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "hourly_pedestrian_counts.csv";
  link.click();
  URL.revokeObjectURL(url);
}
