/**
 * @file robot.js
 * @brief Decoupled 2-Stage Self-Correction Algorithm Simulation for Robotic Arms
 * @copyright Copyright (C) 2026 Zong Ming Yang. All Rights Reserved.
 * @license GNU Affero General Public License v3.0 (AGPL-3.0)
 * * Reference Paper: 
 * "How basic Trigonometric modeling can help predict robotic arm self-correction angle 
 * in microgravity when human input is impaired" - Zong Ming Yang, Jose Salinas (2026).
 */


console.log("🌟 Dynamic Decoupled Solver: Replicating JEI Essay Methodology Perfect Alignment.");

const canvas = document.getElementById('armCanvas');
const ctx = canvas.getContext('2d');

// --- Layout & Work Space Calibration ---
const SCALE = 55;                 // Visual scale multiplier
const base = { x: 450, y: 400 };  // Adjusted Center Base O (0,0) to accommodate wider range
const L1 = 5 * SCALE;             // Main arm link 1 length = 5 units
const L2 = 5 * SCALE;             // Forearm link 2 length = 5 units

let jointF = { x: 0, y: 0 };     
let endE = { x: 0, y: 0 };       
let targetT = { x: 0, y: 0 };    
let ptC = { x: 0, y: 0 }; 
let ptD = { x: 0, y: 0 }; 
let chosenIntersection = { x: 0, y: 0 };

// --- Decoupled Kinematic State Machine ---
let currentAngles = { t1: 0, t2: 0 };
let startAngles = { t1: 0, t2: 0 };
let phase1Angles = { t1: 0, t2: 0 }; 
let phase2Angles = { t1: 0, t2: 0 }; 

let animTimer = 0;
let currentPhase = 0;             // 0: Idle, 1: Phase I (Forearm correction), 2: Phase II (Base sweep)
const ANIM_SPEED = 0.0015;        // Slowed down drastically for granular tracking

// [Essay Sect 3.2]: Find intersection of Circle C1 (Centered at F) and Circle C4 (Centered at O)
function getIntersections(fx, fy, r1, bx, by, r4) {
    const dx = bx - fx;
    const dy = by - fy;
    const d = Math.sqrt(dx * dx + dy * dy);
    if (d > r1 + r4 || d < Math.abs(r1 - r4) || d === 0) return null; 
    const a = (r1 * r1 - r4 * r4 + d * d) / (2 * d);
    const h = Math.sqrt(r1 * r1 - a * a);
    const x2 = fx + (a * dx) / d;
    const y2 = fy + (a * dy) / d;
    return { 
        C: { x: x2 + (h * dy) / d, y: y2 - (h * dx) / d }, 
        D: { x: x2 - (h * dy) / d, y: y2 + (h * dx) / d } 
    };
}

// [Essay Formula 80]: Vector Dot Product Inverse Cosine Angle Calculator
function calculateCorrectionAngle(v1, v2) {
    const dotProduct = v1.x * v2.x + v1.y * v2.y;
    const mag1 = Math.sqrt(v1.x * v1.x + v1.y * v1.y);
    const mag2 = Math.sqrt(v2.x * v2.x + v2.y * v2.y);
    let cosTheta = dotProduct / (mag1 * mag2);
    cosTheta = Math.max(-1, Math.min(1, cosTheta)); 
    
    const angle = Math.acos(cosTheta);
    const crossProduct = v1.x * v2.y - v1.y * v2.x;
    return crossProduct >= 0 ? angle : -angle;
}

function lerpAngle(a, b, t) {
    let delta = b - a;
    while (delta < -Math.PI) delta += Math.PI * 2;
    while (delta > Math.PI) delta -= Math.PI * 2;
    return a + delta * t;
}

function toDegrees(rad) {
    let deg = rad * (180 / Math.PI);
    while (deg < -180) deg += 360;
    while (deg > 180) deg -= 360;
    return deg.toFixed(1);
}

function generateRandomScenario() {
    // Generate Target T within valid operational reach boundaries [2.5 to 8.5]
    const tAngle = Math.random() * Math.PI * 2;
    const tDist = (2.5 + Math.random() * 5.5) * SCALE; 
    targetT = { x: base.x + tDist * Math.cos(tAngle), y: base.y + tDist * Math.sin(tAngle) };

    // Create random astronaut-induced structural operational deviation
    startAngles.t1 = Math.random() * Math.PI * 2;
    startAngles.t2 = (Math.random() * 2 - 1) * (Math.PI * 0.4);

    // Dynamic initial geometry setup
    jointF.x = base.x + L1 * Math.cos(startAngles.t1);
    jointF.y = base.y + L1 * Math.sin(startAngles.t1);
    endE.x = jointF.x + L2 * Math.cos(startAngles.t1 + startAngles.t2);
    endE.y = jointF.y + L2 * Math.sin(startAngles.t1 + startAngles.t2);

    const r4 = Math.sqrt((targetT.x - base.x)**2 + (targetT.y - base.y)**2);
    const intersections = getIntersections(jointF.x, jointF.y, L2, base.x, base.y, r4);

    if (intersections) {
        ptC = intersections.C;
        ptD = intersections.D;

        // Choose optimal intersection path minimizing displacement
        const distC = Math.sqrt((endE.x - ptC.x)**2 + (endE.y - ptC.y)**2);
        const distD = Math.sqrt((endE.x - ptD.x)**2 + (endE.y - ptD.y)**2);
        const bestPt = distC < distD ? ptC : ptD;
        chosenIntersection = bestPt;

        // --- DECOUPLED STAGE 1: Forearm Alignment (Vector FE to FC) ---
        const vec_FE = { x: endE.x - jointF.x, y: endE.y - jointF.y };
        const vec_FC = { x: bestPt.x - jointF.x, y: bestPt.y - jointF.y };
        const alpha = calculateCorrectionAngle(vec_FE, vec_FC);

        phase1Angles.t1 = startAngles.t1;
        phase1Angles.t2 = startAngles.t2 + alpha; 

        // --- DECOUPLED STAGE 2: Whole Arm Rigid Sweep ---
        const tempJointF = { x: base.x + L1 * Math.cos(phase1Angles.t1), y: base.y + L1 * Math.sin(phase1Angles.t1) };
        const tempEndE = { x: tempJointF.x + L2 * Math.cos(phase1Angles.t1 + phase1Angles.t2), y: tempJointF.y + L2 * Math.sin(phase1Angles.t1 + phase1Angles.t2) };
        
        const vec_OE = { x: tempEndE.x - base.x, y: tempEndE.y - base.y };
        const vec_OT = { x: targetT.x - base.x, y: targetT.y - base.y };
        const beta = calculateCorrectionAngle(vec_OE, vec_OT);

        phase2Angles.t1 = phase1Angles.t1 + beta; 
        phase2Angles.t2 = phase1Angles.t2; 

        // Initializing clock parameters
        currentAngles.t1 = startAngles.t1;
        currentAngles.t2 = startAngles.t2;
        currentPhase = 1;
        animTimer = 0;
    } else {
        generateRandomScenario(); 
    }
}

function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // --- State Handler and Separated Interpolation Engine ---
    if (currentPhase === 1) { 
        animTimer += ANIM_SPEED;
        if (animTimer >= 1.0) { animTimer = 0; currentPhase = 2; }
        currentAngles.t1 = startAngles.t1;
        currentAngles.t2 = lerpAngle(startAngles.t2, phase1Angles.t2, animTimer);
    } else if (currentPhase === 2) { 
        animTimer += ANIM_SPEED;
        if (animTimer >= 1.0) { animTimer = 1.0; currentPhase = 3; }
        currentAngles.t1 = lerpAngle(phase1Angles.t1, phase2Angles.t1, animTimer);
        currentAngles.t2 = phase1Angles.t2;
    }

    // Kinematic forward projection to canvas mapping
    jointF.x = base.x + L1 * Math.cos(currentAngles.t1);
    jointF.y = base.y + L1 * Math.sin(currentAngles.t1);
    endE.x = jointF.x + L2 * Math.cos(currentAngles.t1 + currentAngles.t2);
    endE.y = jointF.y + L2 * Math.sin(currentAngles.t1 + currentAngles.t2);

    // =======================================================
    // 🎨 VISUAL LAYER 1: ESSAY BACKGROUND GEOMETRY SCHEMATICS
    // =======================================================
    
    // Circle C3: Maximum reachable envelope
    ctx.beginPath(); ctx.arc(base.x, base.y, L1 + L2, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(244, 67, 54, 0.15)'; ctx.lineWidth = 1; ctx.stroke();
    ctx.fillStyle = 'rgba(244,67,54,0.005)'; ctx.fill();

    // Circle C4: Target range circle trajectory
    const r4 = Math.sqrt((targetT.x - base.x)**2 + (targetT.y - base.y)**2);
    ctx.beginPath(); ctx.arc(base.x, base.y, r4, 0, Math.PI * 2);
    ctx.strokeStyle = '#4caf50'; ctx.lineWidth = 1.5; ctx.stroke();

    // Circle C1: Segment workspace centered at Joint F
    ctx.beginPath(); ctx.arc(jointF.x, jointF.y, L2, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(156, 39, 176, 0.2)'; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);

    // Plotting Intersection Intercepts C and D
    if (currentPhase > 0) {
        ctx.fillStyle = '#ff9800'; ctx.font = 'bold 11px Courier New';
        ctx.beginPath(); ctx.arc(ptC.x, ptC.y, 5, 0, Math.PI*2); ctx.fill();
        ctx.fillText("C (Intersection)", ptC.x + 10, ptC.y - 5);
        ctx.beginPath(); ctx.arc(ptD.x, ptD.y, 5, 0, Math.PI*2); ctx.fill();
        ctx.fillText("D (Intersection)", ptD.x + 10, ptD.y + 12);

        // Reference Line P1 (Origin to Target T)
        ctx.beginPath(); ctx.moveTo(base.x, base.y); ctx.lineTo(targetT.x, targetT.y);
        ctx.strokeStyle = '#e91e63'; ctx.setLineDash([5, 5]); ctx.lineWidth = 1.2; ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = '#e91e63'; ctx.fillText("P1 Ref", (base.x + targetT.x)/2 - 15, (base.y + targetT.y)/2 - 10);
    }

    // Vector Deviation Indicator (E -> T)
    ctx.beginPath(); ctx.moveTo(endE.x, endE.y); ctx.lineTo(targetT.x, targetT.y);
    ctx.strokeStyle = 'rgba(255, 235, 59, 0.4)'; ctx.lineWidth = 1; ctx.stroke();

    // =======================================================
    // 🦾 VISUAL LAYER 2: PHYSICAL MANIPULATOR MECHANICAL LINKS
    // =======================================================
    
    // Main Arm Segment (L1)
    ctx.beginPath(); ctx.moveTo(base.x, base.y); ctx.lineTo(jointF.x, jointF.y);
    ctx.strokeStyle = '#3f51b5'; ctx.lineWidth = 11; ctx.lineCap = 'round'; ctx.stroke();

    // Forearm Segment (L2)
    ctx.beginPath(); ctx.moveTo(jointF.x, jointF.y); ctx.lineTo(endE.x, endE.y);
    ctx.strokeStyle = '#009688'; ctx.lineWidth = 7; ctx.lineCap = 'round'; ctx.stroke();

    // Joint Angular Hinge Arc Overlays
    ctx.beginPath(); ctx.arc(base.x, base.y, 35, 0, currentAngles.t1, currentAngles.t1 < 0);
    ctx.strokeStyle = 'rgba(63, 81, 181, 0.6)'; ctx.lineWidth = 2; ctx.stroke();

    ctx.beginPath(); ctx.arc(jointF.x, jointF.y, 25, currentAngles.t1, currentAngles.t1 + currentAngles.t2, currentAngles.t2 < 0);
    ctx.strokeStyle = 'rgba(0, 150, 136, 0.8)'; ctx.lineWidth = 2; ctx.stroke();

    // Core Nodes
    ctx.fillStyle = '#ffffff'; ctx.beginPath(); ctx.arc(base.x, base.y, 8, 0, Math.PI * 2); ctx.fill(); 
    ctx.font = 'bold 12px Arial'; ctx.fillText("O (Base)", base.x - 55, base.y + 22);
    
    ctx.fillStyle = '#2196f3'; ctx.beginPath(); ctx.arc(jointF.x, jointF.y, 6, 0, Math.PI * 2); ctx.fill(); 
    ctx.fillText("F (Joint)", jointF.x + 12, jointF.y + 4);
    
    ctx.fillStyle = '#ffeb3b'; ctx.beginPath(); ctx.arc(endE.x, endE.y, 6, 0, Math.PI * 2); ctx.fill(); 
    ctx.fillText("E (End)", endE.x + 12, endE.y + 4);
    
    ctx.fillStyle = '#e91e63'; ctx.beginPath(); ctx.arc(targetT.x, targetT.y, 8, 0, Math.PI * 2); ctx.fill(); 
    ctx.fillText("T (Target)", targetT.x + 12, targetT.y + 4);

    // =======================================================
    // 📊 VISUAL LAYER 3: NON-OVERLAPPING ANALYTICS IN PANEL (English)
    // =======================================================
    // Moved completely to bottom-left corner to keep workspace clear
    const panY = 640; 
    ctx.fillStyle = 'rgba(15, 15, 18, 0.9)'; ctx.fillRect(15, panY, 360, 245);
    ctx.strokeStyle = '#33333d'; ctx.lineWidth = 1.5; ctx.strokeRect(15, panY, 360, 245);

    ctx.fillStyle = '#00e5ff'; ctx.font = 'bold 12px sans-serif';
    ctx.fillText("📊 DECOUPLED MATHEMATICAL FRAMEWORK", 30, panY + 25);
    ctx.fillStyle = '#2d2d35'; ctx.fillRect(30, panY + 35, 330, 1);

    ctx.fillStyle = '#eceff1'; ctx.font = '11px monospace';
    ctx.fillText(`Coordinates E : X=${((endE.x - base.x)/SCALE).toFixed(2)}  Y=${((base.y - endE.y)/SCALE).toFixed(2)}`, 30, panY + 55);
    ctx.fillText(`Target Coord T : X=${((targetT.x - base.x)/SCALE).toFixed(2)}  Y=${((base.y - targetT.y)/SCALE).toFixed(2)}`, 30, panY + 75);

    ctx.fillStyle = '#9fa8da';
    ctx.fillText(`Joint 1 Angle (θ1)  : ${toDegrees(currentAngles.t1)}°`, 30, panY + 105);
    ctx.fillStyle = '#80cbc4';
    ctx.fillText(`Joint 2 Angle (θ2)  : ${toDegrees(currentAngles.t2)}°`, 30, panY + 125);

    ctx.fillStyle = '#ff9800'; ctx.font = 'bold 11px sans-serif';
    ctx.fillText("ALGORITHM PROGRESS:", 30, panY + 160);
    
    ctx.font = '11px sans-serif';
    if (currentPhase === 1) {
        ctx.fillStyle = '#ffa726'; ctx.fillText("STAGE I -> Aligning Forearm Vector (α Correction)", 30, panY + 185);
        ctx.fillStyle = '#78909c'; ctx.fillText("Calculating intersection cross points on C4...", 30, panY + 205);
    } else if (currentPhase === 2) {
        ctx.fillStyle = '#29b6f6'; ctx.fillText("STAGE II -> Base Rigid Body Swivel (β Sweep)", 30, panY + 185);
        ctx.fillStyle = '#78909c'; ctx.fillText("Sliding End-effector on target trajectory arc...", 30, panY + 205);
    } else if (currentPhase === 3) {
        ctx.fillStyle = '#66bb6a'; ctx.fillText("● TASK SUCCESS: System Converged To Target", 30, panY + 185);
        ctx.fillStyle = '#aaabb5'; ctx.fillText("Human-induced impairments rectified successfully.", 30, panY + 205);
    }

    requestAnimationFrame(draw);
}

canvas.addEventListener('click', generateRandomScenario);
generateRandomScenario();
draw();