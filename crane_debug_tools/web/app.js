/**
 * Crane Robot Skills Debugger Web Application
 */

class CraneDebugger {
    constructor() {
        this.websocket = null;
        this.isConnected = false;
        this.availableSkills = [];
        this.selectedSkill = null;
        this.worldModel = null;
        this.robots = [];
        
        this.initializeUI();
        this.connectWebSocket();
    }

    initializeUI() {
        // Populate robot ID selector
        const robotSelect = document.getElementById('robotId');
        for (let i = 0; i < 16; i++) {
            const option = document.createElement('option');
            option.value = i;
            option.textContent = `Robot ${i}`;
            robotSelect.appendChild(option);
        }

        // Event listeners
        document.getElementById('executeBtn').addEventListener('click', () => {
            this.executeSelectedSkill();
        });

        document.getElementById('clearLogBtn').addEventListener('click', () => {
            this.clearLog();
        });

        this.addLog('System initialized', 'info');
    }

    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.hostname;
        const port = 8080; // Default WebSocket port
        
        const wsUrl = `${protocol}//${host}:${port}`;
        
        this.addLog(`Connecting to ${wsUrl}...`, 'info');
        
        try {
            this.websocket = new WebSocket(wsUrl);
            
            this.websocket.onopen = () => {
                this.onWebSocketOpen();
            };
            
            this.websocket.onmessage = (event) => {
                this.onWebSocketMessage(event);
            };
            
            this.websocket.onclose = () => {
                this.onWebSocketClose();
            };
            
            this.websocket.onerror = (error) => {
                this.onWebSocketError(error);
            };
        } catch (error) {
            this.addLog(`Failed to create WebSocket connection: ${error.message}`, 'error');
            this.updateConnectionStatus(false);
        }
    }

    onWebSocketOpen() {
        this.isConnected = true;
        this.updateConnectionStatus(true);
        this.addLog('Connected to crane debug server', 'success');
        
        // Request available skills
        this.sendMessage({
            type: 'get_skills'
        });
    }

    onWebSocketMessage(event) {
        try {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        } catch (error) {
            this.addLog(`Failed to parse message: ${error.message}`, 'error');
        }
    }

    onWebSocketClose() {
        this.isConnected = false;
        this.updateConnectionStatus(false);
        this.addLog('Disconnected from server', 'warning');
        
        // Attempt to reconnect after 3 seconds
        setTimeout(() => {
            if (!this.isConnected) {
                this.addLog('Attempting to reconnect...', 'info');
                this.connectWebSocket();
            }
        }, 3000);
    }

    onWebSocketError(error) {
        this.addLog(`WebSocket error: ${error.message || 'Connection failed'}`, 'error');
        this.updateConnectionStatus(false);
    }

    handleMessage(message) {
        switch (message.type) {
            case 'available_skills':
                this.handleAvailableSkills(message.skills);
                break;
            case 'world_model':
                this.handleWorldModel(message);
                break;
            case 'robot_commands':
                this.handleRobotCommands(message.commands);
                break;
            case 'skill_execution_started':
                this.addLog(`Skill execution started: ${message.skill_name} on robot ${message.robot_id}`, 'info');
                break;
            case 'skill_goal_response':
                if (message.accepted) {
                    this.addLog('Skill goal accepted by server', 'success');
                } else {
                    this.addLog('Skill goal rejected by server', 'error');
                }
                break;
            case 'skill_feedback':
                this.addLog(`Feedback: ${message.message}`, 'info');
                break;
            case 'skill_result':
                this.handleSkillResult(message);
                break;
            case 'error':
                this.addLog(`Error: ${message.message}`, 'error');
                break;
            default:
                this.addLog(`Unknown message type: ${message.type}`, 'warning');
        }
    }

    handleAvailableSkills(skills) {
        this.availableSkills = skills;
        this.populateSkillsList(skills);
        this.addLog(`Loaded ${skills.length} available skills`, 'success');
    }

    handleWorldModel(worldModel) {
        this.worldModel = worldModel;
        this.updateFieldVisualization();
        this.updateRobotStatus();
    }

    handleRobotCommands(commands) {
        // Update robot command visualization
        this.addLog(`Received commands for ${commands.length} robots`, 'info');
    }

    handleSkillResult(result) {
        const codeNames = {
            1: 'SUCCEEDED',
            2: 'ABORTED', 
            3: 'CANCELED'
        };
        
        const codeName = codeNames[result.code] || 'UNKNOWN';
        const logType = result.code === 1 ? 'success' : 'error';
        
        this.addLog(`Skill execution ${codeName} (result: ${result.result})`, logType);
    }

    sendMessage(message) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(message));
        } else {
            this.addLog('Cannot send message: not connected', 'error');
        }
    }

    populateSkillsList(skills) {
        const skillsList = document.getElementById('skillsList');
        skillsList.innerHTML = '';
        
        skills.forEach(skill => {
            const skillCard = document.createElement('button');
            skillCard.className = 'btn btn-outline-primary skill-card';
            skillCard.textContent = skill;
            skillCard.addEventListener('click', () => {
                this.selectSkill(skill);
            });
            skillsList.appendChild(skillCard);
        });
    }

    selectSkill(skillName) {
        this.selectedSkill = skillName;
        document.getElementById('selectedSkill').value = skillName;
        document.getElementById('executeBtn').disabled = false;
        
        // Remove active class from all skill cards
        document.querySelectorAll('.skill-card').forEach(card => {
            card.classList.remove('btn-primary');
            card.classList.add('btn-outline-primary');
        });
        
        // Add active class to selected skill
        event.target.classList.remove('btn-outline-primary');
        event.target.classList.add('btn-primary');
        
        this.generateParametersForm(skillName);
        this.addLog(`Selected skill: ${skillName}`, 'info');
    }

    generateParametersForm(skillName) {
        const parametersForm = document.getElementById('parametersForm');
        
        // Define common parameters for different skills
        const skillParameters = {
            'Kick': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' },
                { name: 'kick_power', type: 'number', label: 'Kick Power', min: '0', max: '10', step: '0.1' }
            ],
            'EmplaceRobot': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' },
                { name: 'target_theta', type: 'number', label: 'Target Theta', step: '0.1' }
            ],
            'Sleep': [
                { name: 'duration', type: 'number', label: 'Duration (seconds)', min: '0', step: '0.1', value: '1.0' }
            ],
            'TestMotionPosition': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' }
            ],
            'TestMotionVelocity': [
                { name: 'velocity_x', type: 'number', label: 'Velocity X', step: '0.1' },
                { name: 'velocity_y', type: 'number', label: 'Velocity Y', step: '0.1' }
            ]
        };
        
        const parameters = skillParameters[skillName] || [];
        
        if (parameters.length === 0) {
            parametersForm.innerHTML = '<p class="text-muted">No configurable parameters for this skill</p>';
            return;
        }
        
        let formHTML = '';
        parameters.forEach(param => {
            formHTML += `
                <div class="mb-3">
                    <label for="param_${param.name}" class="form-label">${param.label}</label>
                    <input type="${param.type}" 
                           class="form-control" 
                           id="param_${param.name}" 
                           name="${param.name}"
                           ${param.min ? `min="${param.min}"` : ''}
                           ${param.max ? `max="${param.max}"` : ''}
                           ${param.step ? `step="${param.step}"` : ''}
                           ${param.value ? `value="${param.value}"` : ''}>
                </div>
            `;
        });
        
        parametersForm.innerHTML = formHTML;
    }

    executeSelectedSkill() {
        if (!this.selectedSkill) {
            this.addLog('No skill selected', 'warning');
            return;
        }
        
        const robotId = parseInt(document.getElementById('robotId').value);
        const parameters = this.collectParameters();
        
        const message = {
            type: 'execute_skill',
            skill_name: this.selectedSkill,
            robot_id: robotId,
            parameters: parameters
        };
        
        this.sendMessage(message);
        this.addLog(`Executing ${this.selectedSkill} on robot ${robotId}...`, 'info');
    }

    collectParameters() {
        const parametersForm = document.getElementById('parametersForm');
        const inputs = parametersForm.querySelectorAll('input');
        const parameters = [];
        
        inputs.forEach(input => {
            if (input.value.trim() !== '') {
                parameters.push({
                    name: input.name,
                    value: input.value
                });
            }
        });
        
        return parameters;
    }

    updateFieldVisualization() {
        if (!this.worldModel) return;
        
        const field = document.getElementById('fieldViz');
        const fieldRect = field.getBoundingClientRect();
        
        // Clear previous robots and ball
        field.querySelectorAll('.robot, .ball').forEach(el => el.remove());
        
        // Field dimensions (SSL field is 12m x 9m)
        const fieldWidth = 12; // meters
        const fieldHeight = 9; // meters
        
        // Scale world coordinates to screen coordinates
        const scaleX = fieldRect.width / fieldWidth;
        const scaleY = fieldRect.height / fieldHeight;
        
        // Draw ball
        if (this.worldModel.ball) {
            const ball = document.createElement('div');
            ball.className = 'ball';
            ball.style.left = `${(this.worldModel.ball.x + fieldWidth/2) * scaleX}px`;
            ball.style.top = `${(fieldHeight/2 - this.worldModel.ball.y) * scaleY}px`;
            field.appendChild(ball);
        }
        
        // Draw robots
        if (this.worldModel.robots) {
            this.worldModel.robots.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.title = `Robot ${robot.id}`;
                
                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';
                
                field.appendChild(robotEl);
            });
        }
    }

    updateRobotStatus() {
        if (!this.worldModel || !this.worldModel.robots) {
            return;
        }
        
        const statusContainer = document.getElementById('robotStatus');
        let statusHTML = '';
        
        this.worldModel.robots.forEach(robot => {
            statusHTML += `
                <div class="mb-2 p-2 border rounded">
                    <strong>Robot ${robot.id}</strong><br>
                    <small>
                        Pos: (${robot.x.toFixed(2)}, ${robot.y.toFixed(2)})<br>
                        Vel: (${robot.vx.toFixed(2)}, ${robot.vy.toFixed(2)})<br>
                        θ: ${robot.theta.toFixed(2)} rad
                    </small>
                </div>
            `;
        });
        
        if (statusHTML === '') {
            statusHTML = '<p class="text-muted">No robot data available</p>';
        }
        
        statusContainer.innerHTML = statusHTML;
    }

    updateConnectionStatus(connected) {
        const statusIndicator = document.getElementById('connectionStatus');
        const statusText = document.getElementById('connectionText');
        
        if (connected) {
            statusIndicator.className = 'status-indicator connected';
            statusText.textContent = 'Connected';
        } else {
            statusIndicator.className = 'status-indicator disconnected';
            statusText.textContent = 'Disconnected';
        }
    }

    addLog(message, type = 'info') {
        const logContainer = document.getElementById('logContainer');
        const timestamp = new Date().toLocaleTimeString();
        
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;
        logEntry.innerHTML = `<span class="text-muted">[${timestamp}]</span> ${message}`;
        
        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;
        
        // Limit log entries to 1000
        const entries = logContainer.children;
        if (entries.length > 1000) {
            logContainer.removeChild(entries[0]);
        }
    }

    clearLog() {
        document.getElementById('logContainer').innerHTML = '';
        this.addLog('Log cleared', 'info');
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new CraneDebugger();
});