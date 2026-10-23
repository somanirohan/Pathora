// Pathora Frontend Application Logic
// Handles Graph Canvas Rendering, Interactive Editing, API Communication, Animation, and Telemetry

const CONFIG = {
  API_BASE: 'http://localhost:8080/api',
  CANVAS_PAD: 60,
  NODE_RADIUS: 9,
  EXIT_RADIUS: 14,
};

const STATE = {
  activeFloor: 0,
  activeMode: 'navigation', // 'navigation' | 'crowd_flow' | 'evacuation'
  activeTool: 'select',     // 'select' | 'start' | 'target' | 'hazard' | 'wall' | 'crowd'
  nodes: new Map(),
  edges: [],
  exitNodeIds: [],
  startNodeId: 1,
  targetNodeId: 15,
  activePath: [],
  exploredSequence: [],
  animatingExploration: false,
  crowdAgents: [],
  simIntervalId: null,
  simRunning: false,
  metrics: null
};

// DOM Elements
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const tooltip = document.getElementById('nodeTooltip');

// Initialize Application
async function initApp() {
  setupCanvasResize();
  setupEventListeners();
  await fetchGraphData();
  render();
}

// Window resizing for Retina/High-DPI displays
function setupCanvasResize() {
  function resize() {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.scale(dpr, dpr);
    render();
  }
  window.addEventListener('resize', resize);
  resize();
}

// Event Listeners setup
function setupEventListeners() {
  // Mode switcher tabs
  document.querySelectorAll('.mode-tab').forEach(tab => {
    tab.addEventListener('click', (e) => {
      document.querySelectorAll('.mode-tab').forEach(t => t.classList.remove('active'));
      const targetTab = e.currentTarget;
      targetTab.classList.add('active');
      STATE.activeMode = targetTab.dataset.mode;
      updateModeUI();
    });
  });

  // Floor switcher
  document.querySelectorAll('.floor-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.floor-btn').forEach(b => b.classList.remove('active'));
      e.target.classList.add('active');
      STATE.activeFloor = parseInt(e.target.dataset.floor, 10);
      render();
    });
  });

  // Tools
  document.querySelectorAll('.tool-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
      const targetBtn = e.currentTarget;
      targetBtn.classList.add('active');
      STATE.activeTool = targetBtn.dataset.tool;
    });
  });

  // Slider controls display update
  ['weightDistance', 'weightRisk', 'weightCongestion'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('input', (e) => {
        document.getElementById(`${id}Val`).innerText = e.target.value;
      });
    }
  });

  // Canvas interaction
  canvas.addEventListener('click', handleCanvasClick);
  canvas.addEventListener('mousemove', handleCanvasHover);

  // Compute Path button
  document.getElementById('btnCalculate').addEventListener('click', executePathfinding);

  // Compare Algorithms button
  document.getElementById('btnCompare').addEventListener('click', executeBenchmark);

  // Simulation buttons
  document.getElementById('btnSpawnCrowd').addEventListener('click', spawnCrowd);
  document.getElementById('btnSimStep').addEventListener('click', stepSimulation);
  document.getElementById('btnSimToggle').addEventListener('click', toggleSimulationLoop);
  document.getElementById('btnResetEnv').addEventListener('click', resetEnvironment);

  // Telemetry drawer collapse toggle
  document.getElementById('trayHeader').addEventListener('click', () => {
    document.getElementById('telemetryTray').classList.toggle('collapsed');
  });
}

function updateModeUI() {
  const evacBanner = document.getElementById('evacuationAlertBanner');
  const crowdOptions = document.getElementById('crowdOptionsGroup');
  const algSelect = document.getElementById('algSelect');

  if (STATE.activeMode === 'evacuation') {
    evacBanner.style.display = 'flex';
    algSelect.value = 'adaptive';
    document.getElementById('statusDot').className = 'status-dot danger';
    document.getElementById('systemStatusText').innerText = 'EMERGENCY EVACUATION ACTIVE';
  } else {
    evacBanner.style.display = 'none';
    document.getElementById('statusDot').className = 'status-dot';
    document.getElementById('systemStatusText').innerText = 'Operational (Idle)';
  }
}

// API Calls
async function fetchGraphData() {
  try {
    const res = await fetch(`${CONFIG.API_BASE}/graph`);
    const data = await res.json();
    STATE.nodes.clear();
    data.nodes.forEach(n => STATE.nodes.set(n.id, n));
    STATE.edges = data.edges;
    STATE.exitNodeIds = data.exitNodeIds;

    // Default start/target check
    if (!STATE.nodes.has(STATE.startNodeId)) STATE.startNodeId = data.nodes[0]?.id || 1;
    if (!STATE.nodes.has(STATE.targetNodeId)) STATE.targetNodeId = data.nodes[data.nodes.length - 1]?.id || 10;
    
    document.getElementById('nodeCountBadge').innerText = `${data.nodeCount} Nodes | ${data.edgeCount} Edges`;
    render();
  } catch (err) {
    console.error('Failed to fetch graph:', err);
  }
}

async function executePathfinding() {
  const alg = document.getElementById('algSelect').value;
  const mobility = document.getElementById('mobilitySelect').value;
  const wDist = parseFloat(document.getElementById('weightDistance').value);
  const wRisk = parseFloat(document.getElementById('weightRisk').value);
  const wCrowd = parseFloat(document.getElementById('weightCongestion').value);
  const avoidStairs = document.getElementById('chkAvoidStairs').checked;

  const payload = {
    startNodeId: STATE.startNodeId,
    targetNodeId: STATE.activeMode === 'evacuation' ? -1 : STATE.targetNodeId,
    algorithm: alg,
    mode: STATE.activeMode,
    mobility: mobility,
    weightDistance: wDist,
    weightRisk: wRisk,
    weightCongestion: wCrowd,
    avoidStairs: avoidStairs
  };

  try {
    document.getElementById('btnCalculate').innerText = 'Routing...';
    const res = await fetch(`${CONFIG.API_BASE}/navigate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    const result = await res.json();
    document.getElementById('btnCalculate').innerText = 'Calculate Optimal Path';

    if (result.success) {
      STATE.activePath = result.path;
      STATE.exploredSequence = result.exploredNodeSequence || [];
      STATE.metrics = result.metrics;
      displayMetrics(result.metrics, result.statusMessage);
      animateExploration();
    } else {
      alert(`Routing Failed: ${result.statusMessage}`);
    }
  } catch (err) {
    console.error('Pathfinding query error:', err);
    document.getElementById('btnCalculate').innerText = 'Calculate Optimal Path';
  }
}

async function executeBenchmark() {
  const payload = {
    startNodeId: STATE.startNodeId,
    targetNodeId: STATE.targetNodeId,
    mode: STATE.activeMode
  };

  try {
    const res = await fetch(`${CONFIG.API_BASE}/compare`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    const data = await res.json();
    displayBenchmarkTable(data.benchmark);
  } catch (err) {
    console.error('Benchmark execution failed:', err);
  }
}

async function spawnCrowd() {
  const count = parseInt(document.getElementById('spawnCountInput').value, 10) || 20;
  const mobility = document.getElementById('mobilitySelect').value;
  try {
    const res = await fetch(`${CONFIG.API_BASE}/simulation/spawn`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ count, profile: mobility })
    });
    await fetchGraphData();
    stepSimulation();
  } catch (err) {
    console.error('Error spawning crowd:', err);
  }
}

async function stepSimulation() {
  try {
    const res = await fetch(`${CONFIG.API_BASE}/simulation/step`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ spreadHazard: STATE.activeMode === 'evacuation' })
    });
    const simData = await res.json();
    STATE.crowdAgents = simData.agents;

    document.getElementById('metricSimTick').innerText = simData.tick;
    document.getElementById('metricEvacuated').innerText = simData.safeEvacuated;
    document.getElementById('metricTrapped').innerText = simData.trapped;

    await fetchGraphData();
    render();
  } catch (err) {
    console.error('Error during sim step:', err);
  }
}

function toggleSimulationLoop() {
  const btn = document.getElementById('btnSimToggle');
  if (STATE.simRunning) {
    clearInterval(STATE.simIntervalId);
    STATE.simRunning = false;
    btn.innerText = 'Run Dynamic Sim';
    btn.classList.remove('btn-danger');
    btn.classList.add('btn-secondary');
  } else {
    STATE.simRunning = true;
    btn.innerText = 'Pause Sim';
    btn.classList.remove('btn-secondary');
    btn.classList.add('btn-danger');
    STATE.simIntervalId = setInterval(stepSimulation, 650);
  }
}

async function resetEnvironment() {
  if (STATE.simRunning) toggleSimulationLoop();
  await fetch(`${CONFIG.API_BASE}/reset`, { method: 'POST' });
  STATE.activePath = [];
  STATE.exploredSequence = [];
  STATE.crowdAgents = [];
  await fetchGraphData();
  render();
}

// Display Telemetry & Metrics
function displayMetrics(m, statusMsg) {
  document.getElementById('metricAlgName').innerText = m.algorithmName;
  document.getElementById('metricExecTime').innerText = `${m.executionTimeMicroseconds} μs`;
  document.getElementById('metricNodesExplored').innerText = `${m.nodesExpanded} (Visited: ${m.nodesVisited})`;
  document.getElementById('metricPathDist').innerText = `${m.pathDistance.toFixed(1)} m`;
  document.getElementById('metricCompositeCost').innerText = m.compositeCost.toFixed(1);
  document.getElementById('metricComplexity').innerText = `${m.theoreticalTimeComplexity} | ${m.theoreticalSpaceComplexity}`;
  
  const reasonBox = document.getElementById('adaptiveReasonBox');
  if (m.selectionReason) {
    reasonBox.style.display = 'block';
    document.getElementById('adaptiveReasonText').innerText = m.selectionReason;
  } else {
    reasonBox.style.display = 'none';
  }

  // Ensure telemetry drawer is expanded
  document.getElementById('telemetryTray').classList.remove('collapsed');
}

function displayBenchmarkTable(results) {
  const tbody = document.getElementById('benchmarkTableBody');
  tbody.innerHTML = '';

  results.forEach(item => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><strong>${item.algorithm}</strong></td>
      <td><span class="badge-tag ${item.timeMicroseconds < 200 ? 'green' : 'blue'}">${item.timeMicroseconds} μs</span></td>
      <td>${item.nodesExpanded}</td>
      <td>${item.pathDistance.toFixed(1)} m</td>
      <td>${item.compositeCost.toFixed(1)}</td>
      <td><code>${item.timeComplexity}</code></td>
      <td><code>${item.spaceComplexity}</code></td>
    `;
    tbody.appendChild(tr);
  });

  document.getElementById('benchmarkSection').style.display = 'block';
  document.getElementById('telemetryTray').classList.remove('collapsed');
}

// Animation of search exploration
function animateExploration() {
  let step = 0;
  STATE.animatingExploration = true;
  const sequence = STATE.exploredSequence;

  function frame() {
    if (step < sequence.length) {
      step += Math.max(1, Math.floor(sequence.length / 30));
      renderExplorationProgress(sequence.slice(0, step));
      requestAnimationFrame(frame);
    } else {
      STATE.animatingExploration = false;
      render();
    }
  }
  frame();
}

// Canvas Interaction (Click / Hover)
function handleCanvasClick(e) {
  const rect = canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  let clickedNode = null;
  for (const node of STATE.nodes.values()) {
    if (node.floor !== STATE.activeFloor) continue;
    const dist = Math.hypot(node.x - clickX, node.y - clickY);
    if (dist <= CONFIG.NODE_RADIUS + 6) {
      clickedNode = node;
      break;
    }
  }

  if (!clickedNode) return;

  if (STATE.activeTool === 'start') {
    STATE.startNodeId = clickedNode.id;
    render();
  } else if (STATE.activeTool === 'target') {
    STATE.targetNodeId = clickedNode.id;
    render();
  } else if (STATE.activeTool === 'hazard') {
    const newHazard = clickedNode.hazardLevel > 0.5 ? 0.0 : 1.0;
    updateNodeOnServer(clickedNode.id, { hazard: newHazard });
  } else if (STATE.activeTool === 'wall') {
    const walkable = !clickedNode.isWalkable;
    updateNodeOnServer(clickedNode.id, { walkable });
  } else if (STATE.activeTool === 'crowd') {
    const newCrowd = clickedNode.crowdDensity > 0.5 ? 0.0 : 0.8;
    updateNodeOnServer(clickedNode.id, { crowd: newCrowd });
  }
}

async function updateNodeOnServer(nodeId, data) {
  try {
    await fetch(`${CONFIG.API_BASE}/node/update`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ nodeId, ...data })
    });
    await fetchGraphData();
    render();
  } catch (err) {
    console.error('Failed to update node:', err);
  }
}

function handleCanvasHover(e) {
  const rect = canvas.getBoundingClientRect();
  const mouseX = e.clientX - rect.left;
  const mouseY = e.clientY - rect.top;

  let hoveredNode = null;
  for (const node of STATE.nodes.values()) {
    if (node.floor !== STATE.activeFloor) continue;
    if (Math.hypot(node.x - mouseX, node.y - mouseY) <= CONFIG.NODE_RADIUS + 6) {
      hoveredNode = node;
      break;
    }
  }

  if (hoveredNode) {
    tooltip.style.display = 'block';
    tooltip.style.left = `${e.clientX + 14}px`;
    tooltip.style.top = `${e.clientY + 14}px`;
    tooltip.innerHTML = `
      <strong>ID: ${hoveredNode.id}</strong> - ${hoveredNode.label}<br/>
      Floor: ${hoveredNode.floor} | Walkable: ${hoveredNode.isWalkable ? 'Yes' : 'No'}<br/>
      Hazard: ${(hoveredNode.hazardLevel * 100).toFixed(0)}% | Crowd: ${(hoveredNode.crowdDensity * 100).toFixed(0)}%
    `;
  } else {
    tooltip.style.display = 'none';
  }
}

// Rendering Engine
function render() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);

  drawGridBackground(rect.width, rect.height);
  drawEdges();
  drawNodes();
  drawActivePath();
  drawCrowdAgents();
}

function drawGridBackground(w, h) {
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
  ctx.lineWidth = 1;
  const step = 40;
  for (let x = 0; x < w; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = 0; y < h; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
}

function drawEdges() {
  ctx.lineWidth = 2.5;
  for (const e of STATE.edges) {
    const u = STATE.nodes.get(e.from);
    const v = STATE.nodes.get(e.to);
    if (!u || !v) continue;
    if (u.floor !== STATE.activeFloor || v.floor !== STATE.activeFloor) continue;

    ctx.beginPath();
    ctx.moveTo(u.x, u.y);
    ctx.lineTo(v.x, v.y);

    if (e.isStairs) {
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.4)';
      ctx.setLineDash([4, 4]);
    } else if (e.isElevator) {
      ctx.strokeStyle = 'rgba(139, 92, 246, 0.4)';
      ctx.setLineDash([2, 4]);
    } else {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
      ctx.setLineDash([]);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawNodes() {
  for (const n of STATE.nodes.values()) {
    if (n.floor !== STATE.activeFloor) continue;

    ctx.save();
    ctx.beginPath();
    ctx.arc(n.x, n.y, CONFIG.NODE_RADIUS, 0, Math.PI * 2);

    // Hazard overlay glow
    if (n.hazardLevel > 0.1) {
      ctx.fillStyle = `rgba(244, 63, 94, ${0.4 + n.hazardLevel * 0.5})`;
      ctx.shadowColor = '#f43f5e';
      ctx.shadowBlur = 12 * n.hazardLevel;
    } else if (n.crowdDensity > 0.2) {
      // Crowd heat gradient
      ctx.fillStyle = `rgba(245, 158, 11, ${0.3 + n.crowdDensity * 0.6})`;
      ctx.shadowColor = '#f59e0b';
      ctx.shadowBlur = 8 * n.crowdDensity;
    } else if (!n.isWalkable) {
      ctx.fillStyle = '#374151'; // Wall
    } else if (n.type === 3) {
      // EMERGENCY EXIT
      ctx.fillStyle = '#10b981';
      ctx.shadowColor = '#10b981';
      ctx.shadowBlur = 15;
    } else if (n.type === 4) {
      // Stairway
      ctx.fillStyle = '#f59e0b';
    } else if (n.type === 5) {
      // Elevator
      ctx.fillStyle = '#8b5cf6';
    } else if (n.type === 6) {
      // Ramp
      ctx.fillStyle = '#06b6d4';
    } else {
      ctx.fillStyle = '#1f2937';
    }

    ctx.fill();
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.stroke();
    ctx.restore();

    // Start / Target Indicators
    if (n.id === STATE.startNodeId) {
      ctx.beginPath();
      ctx.arc(n.x, n.y, CONFIG.NODE_RADIUS + 4, 0, Math.PI * 2);
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 3;
      ctx.stroke();
    } else if (n.id === STATE.targetNodeId && STATE.activeMode !== 'evacuation') {
      ctx.beginPath();
      ctx.arc(n.x, n.y, CONFIG.NODE_RADIUS + 4, 0, Math.PI * 2);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 3;
      ctx.stroke();
    }
  }
}

function drawActivePath() {
  if (!STATE.activePath || STATE.activePath.length < 2) return;

  ctx.save();
  ctx.lineWidth = 5;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.strokeStyle = STATE.activeMode === 'evacuation' ? '#10b981' : '#06b6d4';
  ctx.shadowColor = STATE.activeMode === 'evacuation' ? '#10b981' : '#06b6d4';
  ctx.shadowBlur = 16;

  ctx.beginPath();
  let pathStarted = false;

  for (let i = 0; i < STATE.activePath.length; ++i) {
    const node = STATE.nodes.get(STATE.activePath[i]);
    if (!node || node.floor !== STATE.activeFloor) {
      pathStarted = false;
      continue;
    }
    if (!pathStarted) {
      ctx.moveTo(node.x, node.y);
      pathStarted = true;
    } else {
      ctx.lineTo(node.x, node.y);
    }
  }
  ctx.stroke();
  ctx.restore();
}

function renderExplorationProgress(nodesList) {
  render();
  ctx.save();
  ctx.fillStyle = 'rgba(6, 182, 212, 0.3)';
  for (const id of nodesList) {
    const n = STATE.nodes.get(id);
    if (n && n.floor === STATE.activeFloor) {
      ctx.beginPath();
      ctx.arc(n.x, n.y, CONFIG.NODE_RADIUS + 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }
  ctx.restore();
}

function drawCrowdAgents() {
  for (const a of STATE.crowdAgents) {
    if (a.evacuated) continue;
    const node = STATE.nodes.get(a.currentNodeId);
    if (!node || node.floor !== STATE.activeFloor) continue;

    ctx.save();
    ctx.beginPath();
    ctx.arc(node.x + (Math.sin(a.id) * 6), node.y + (Math.cos(a.id) * 6), 4.5, 0, Math.PI * 2);
    ctx.fillStyle = a.injured ? '#ef4444' : '#38bdf8';
    ctx.shadowColor = a.injured ? '#ef4444' : '#38bdf8';
    ctx.shadowBlur = 6;
    ctx.fill();
    ctx.restore();
  }
}

window.addEventListener('DOMContentLoaded', initApp);
