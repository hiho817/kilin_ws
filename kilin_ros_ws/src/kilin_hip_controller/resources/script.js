class Knob {
    constructor(canvasId, valueInputId, label, zeroAngle = Math.PI / 2, cwPositive = true) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.input = document.getElementById(valueInputId);
        this.label = label;
        
        this.value = 0;
        this.min = -Math.PI;  // -3.14 rad (1 rotation range)
        this.max = Math.PI;   // +3.14 rad (1 rotation range)
        this.isDragging = false;
        
        // Knob orientation
        this.zeroAngle = zeroAngle;  // Angle where value = 0 (in normalized system)
        this.cwPositive = cwPositive;  // true: CW increases value, false: CCW increases value
        
        this.centerX = this.canvas.width / 2;
        this.centerY = this.canvas.height / 2;
        this.radius = 80;
        
        this.setupEventListeners();
        this.draw();
    }

    setupEventListeners() {
        this.canvas.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        this.canvas.addEventListener('mouseup', () => this.handleMouseUp());
        this.canvas.addEventListener('mouseleave', () => this.handleMouseUp());
        
        // Touch events for mobile
        this.canvas.addEventListener('touchstart', (e) => this.handleTouchStart(e));
        this.canvas.addEventListener('touchmove', (e) => this.handleTouchMove(e));
        this.canvas.addEventListener('touchend', () => this.handleMouseUp());
        
        // Input field change
        this.input.addEventListener('change', (e) => {
            const val = parseFloat(e.target.value);
            this.setValue(Math.max(this.min, Math.min(this.max, val)));
            this.publishValue();
        });
    }

    handleMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        const dx = x - this.centerX;
        const dy = y - this.centerY;
        const dist = Math.sqrt(dx * dx + dy * dy);
        
        if (dist < this.radius + 10) {
            this.isDragging = true;
        }
    }

    handleTouchStart(e) {
        if (e.touches.length !== 1) return;
        const rect = this.canvas.getBoundingClientRect();
        const x = e.touches[0].clientX - rect.left;
        const y = e.touches[0].clientY - rect.top;
        
        const dx = x - this.centerX;
        const dy = y - this.centerY;
        const dist = Math.sqrt(dx * dx + dy * dy);
        
        if (dist < this.radius + 10) {
            this.isDragging = true;
            e.preventDefault();
        }
    }

    handleMouseMove(e) {
        if (!this.isDragging) return;
        
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.updateValueFromPosition(x, y);
    }

    handleTouchMove(e) {
        if (!this.isDragging || e.touches.length !== 1) return;
        
        const rect = this.canvas.getBoundingClientRect();
        const x = e.touches[0].clientX - rect.left;
        const y = e.touches[0].clientY - rect.top;
        
        this.updateValueFromPosition(x, y);
        e.preventDefault();
    }

    handleMouseUp() {
        if (this.isDragging) {
            this.isDragging = false;
            // publishValue already called during drag, no need to call again
        }
    }

    updateValueFromPosition(x, y) {
        const dx = x - this.centerX;
        const dy = y - this.centerY;
        
        let angle = Math.atan2(dy, dx);
        // Normalize: subtract π/2 so 0 is at top
        angle = angle - Math.PI / 2;
        
        if (angle < 0) angle += 2 * Math.PI;
        
        // Calculate value relative to zero angle
        let relativeAngle = angle - this.zeroAngle;
        
        // Normalize to [-π, π]
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
        
        // Apply direction: if not CW positive, negate (flip for CCW)
        if (!this.cwPositive) {
            relativeAngle = -relativeAngle;
        }
        
        // Clamp to valid range
        this.value = Math.max(this.min, Math.min(this.max, relativeAngle));
        
        this.input.value = this.value.toFixed(2);
        this.draw();
        this.publishValue();
    }

    setValue(value) {
        this.value = Math.max(this.min, Math.min(this.max, value));
        this.input.value = this.value.toFixed(2);
        this.draw();
    }

    draw() {
        // Clear canvas
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Draw background circle
        this.ctx.fillStyle = '#f0f4ff';
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, this.radius, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // Draw border
        this.ctx.strokeStyle = '#667eea';
        this.ctx.lineWidth = 3;
        this.ctx.stroke();
        
        // Draw tick marks
        this.ctx.strokeStyle = '#ccc';
        this.ctx.lineWidth = 1;
        for (let i = 0; i < 12; i++) {
            const angle = (i / 12) * 2 * Math.PI - Math.PI / 2;
            const x1 = this.centerX + (this.radius - 10) * Math.cos(angle);
            const y1 = this.centerY + (this.radius - 10) * Math.sin(angle);
            const x2 = this.centerX + (this.radius - 5) * Math.cos(angle);
            const y2 = this.centerY + (this.radius - 5) * Math.sin(angle);
            
            this.ctx.beginPath();
            this.ctx.moveTo(x1, y1);
            this.ctx.lineTo(x2, y2);
            this.ctx.stroke();
        }
        
        // Draw value pointer/indicator
        // Calculate display angle: start at zero angle, then add value
        // If CW positive, value rotates CW; if CCW positive, value rotates CCW
        let displayAngle = this.zeroAngle + (this.cwPositive ? this.value : -this.value);
        
        // Add π/2 to convert from normalized coordinates (0 at top) to canvas coordinates (0 at right)
        displayAngle += Math.PI / 2;
        
        // Normalize display angle to [0, 2π]
        displayAngle = displayAngle % (2 * Math.PI);
        if (displayAngle < 0) displayAngle += 2 * Math.PI;
        
        // Draw pointer line
        this.ctx.strokeStyle = '#667eea';
        this.ctx.lineWidth = 4;
        this.ctx.beginPath();
        this.ctx.moveTo(this.centerX, this.centerY);
        this.ctx.lineTo(
            this.centerX + (this.radius - 15) * Math.cos(displayAngle),
            this.centerY + (this.radius - 15) * Math.sin(displayAngle)
        );
        this.ctx.stroke();
        
        // Draw center circle
        this.ctx.fillStyle = '#667eea';
        this.ctx.beginPath();
        this.ctx.arc(this.centerX, this.centerY, 8, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // Draw value arc
        this.ctx.strokeStyle = '#764ba2';
        this.ctx.lineWidth = 6;
        this.ctx.beginPath();
        // Convert zero angle to canvas coordinates
        const arcStartAngle = this.zeroAngle + Math.PI / 2;
        this.ctx.arc(
            this.centerX, this.centerY, this.radius - 10,
            arcStartAngle,
            displayAngle,
            !this.cwPositive  // Direction: true (CCW) if CCW positive, false (CW) if CW positive
        );
        this.ctx.stroke();
    }

    publishValue() {
        // Send value to the server
        fetch(`/api/hip/set`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                module: this.label,
                position: this.value
            })
        }).catch(err => {
            console.error('Failed to publish:', err);
            updateStatus('Error publishing value', 'error');
        });
    }

    getValue() {
        return this.value;
    }
}

// Global knob instances
let knobs = {};

function initializeKnobs() {
    // FL: top-left, zero pointing left (π/2), CCW positive (flipped X-axis)
    knobs.fl = new Knob('knob-fl', 'value-fl', 'A', Math.PI / 2, false);
    
    // FR: top-right, zero pointing right (3π/2), CW positive (flipped X-axis)
    knobs.fr = new Knob('knob-fr', 'value-fr', 'B', 3 * Math.PI / 2, true);
    
    // RL: bottom-left, zero pointing left (π/2), CCW positive
    knobs.rl = new Knob('knob-rl', 'value-rl', 'C', Math.PI / 2, false);
    
    // RR: bottom-right, zero pointing right (3π/2), CW positive
    knobs.rr = new Knob('knob-rr', 'value-rr', 'D', 3 * Math.PI / 2, true);
}

function updateStatus(message, type = 'info') {
    const statusEl = document.getElementById('status');
    statusEl.textContent = message;
    statusEl.className = 'status-text';
    
    if (type === 'error') {
        statusEl.style.color = '#f56565';
        statusEl.style.background = '#fff5f5';
    } else if (type === 'success') {
        statusEl.style.color = '#48bb78';
        statusEl.style.background = '#f0fdf4';
    } else {
        statusEl.style.color = '#667eea';
        statusEl.style.background = '#f0f4ff';
    }
}

function resetAll() {
    Object.values(knobs).forEach(knob => {
        knob.setValue(0);
        knob.publishValue();
    });
    updateStatus('Reset all to 0', 'success');
}

function homePosition() {
    // Home position: all hips at 0
    Object.values(knobs).forEach(knob => {
        knob.setValue(0);
        knob.publishValue();
    });
    updateStatus('Moving to home position', 'success');
}

document.addEventListener('DOMContentLoaded', () => {
    initializeKnobs();
    
    // Setup button listeners
    document.getElementById('reset-btn').addEventListener('click', resetAll);
    document.getElementById('home-btn').addEventListener('click', homePosition);
    
    // Check server connection
    fetch('/api/health')
        .then(r => r.json())
        .then(data => {
            updateStatus('Connected', 'success');
        })
        .catch(err => {
            updateStatus('Connection failed', 'error');
            console.error('Connection error:', err);
        });
});

// Handle window resize
window.addEventListener('resize', () => {
    Object.values(knobs).forEach(knob => {
        knob.draw();
    });
});
