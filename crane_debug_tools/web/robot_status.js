/**
 * Crane Robot Status Monitor
 * WebSocketを使用してロボットの状態をリアルタイムで監視・表示
 */

class RobotStatusMonitor {
    constructor() {
        this.websocket = null;
        this.isConnected = false;
        this.robots = new Map(); // robotId -> robot data
        this.worldModel = null;
        this.robotFeedbacks = new Map(); // robotId -> feedback data
        this.diagnostics = new Map(); // robotId -> diagnostics data
        this.executingSkills = new Map(); // robotId -> skill info
        this.selectedRobotId = null;
        this.sidePanelOpen = false;
        this.teamFilter = 'all'; // 'all', 'ours', 'theirs'
        
        // Update intervals
        this.lastGridUpdate = 0;
        this.gridUpdateInterval = 500; // Update grid every 500ms
        this.lastPanelUpdate = 0;
        this.panelUpdateInterval = 100; // Update panel every 100ms for real-time feel
        
        this.initializeUI();
        this.connectWebSocket();
    }

    initializeUI() {
        // Team filter event listeners
        document.querySelectorAll('input[name="teamFilter"]').forEach(radio => {
            radio.addEventListener('change', (e) => {
                if (e.target.checked) {
                    this.teamFilter = e.target.id.replace('filter', '').toLowerCase();
                    if (this.teamFilter === 'all') this.teamFilter = 'all';
                    this.updateRobotGrid();
                }
            });
        });

        // Side panel controls
        document.getElementById('closePanelBtn').addEventListener('click', () => {
            this.closeSidePanel();
        });

        document.getElementById('overlay').addEventListener('click', () => {
            this.closeSidePanel();
        });

        // Close panel with Escape key
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.sidePanelOpen) {
                this.closeSidePanel();
            }
        });

        this.addLog('Robot Status Monitor initialized');
    }

    connectWebSocket() {
        const wsUrl = `ws://${window.location.hostname}:8091`;
        this.addLog(`Connecting to ${wsUrl}...`);

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
            this.addLog(`WebSocket connection failed: ${error.message}`);
            this.updateConnectionStatus(false);
        }
    }

    onWebSocketOpen() {
        this.isConnected = true;
        this.updateConnectionStatus(true);
        this.addLog('Connected to crane debug server');

        // Activate SimpleAI session
        this.sendMessage({
            type: 'activate_simple_ai'
        });
    }

    onWebSocketMessage(event) {
        try {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        } catch (error) {
            this.addLog(`Failed to parse message: ${error.message}`);
        }
    }

    onWebSocketClose() {
        this.isConnected = false;
        this.updateConnectionStatus(false);
        this.addLog('Disconnected from server');

        // Attempt to reconnect after 3 seconds
        setTimeout(() => {
            if (!this.isConnected) {
                this.addLog('Attempting to reconnect...');
                this.connectWebSocket();
            }
        }, 3000);
    }

    onWebSocketError(error) {
        this.addLog(`WebSocket error: ${error.message || 'Connection failed'}`);
        this.updateConnectionStatus(false);
    }

    handleMessage(message) {
        switch (message.type) {
            case 'world_model':
                this.handleWorldModel(message);
                break;
            case 'robot_commands':
                this.handleRobotCommands(message);
                break;
            case 'robot_diagnostics':
                this.handleRobotDiagnostics(message);
                break;
            case 'robot_feedback':
                this.handleRobotFeedback(message);
                break;
            case 'simple_ai_activated':
                this.addLog('SimpleAI session activated');
                break;
            default:
                // Ignore unknown message types for this interface
                break;
        }
    }

    handleWorldModel(worldModel) {
        this.worldModel = worldModel;
        
        // Update robot data from world model
        if (worldModel.robots_ours) {
            worldModel.robots_ours.forEach(robot => {
                this.updateRobotData(robot.id, robot, 'ours');
            });
        }
        
        if (worldModel.robots_theirs) {
            worldModel.robots_theirs.forEach(robot => {
                this.updateRobotData(robot.id, robot, 'theirs');
            });
        }
        
        this.scheduleGridUpdate();
        
        if (this.sidePanelOpen) {
            this.schedulePanelUpdate();
        }
    }

    handleRobotCommands(commands) {
        if (commands.commands) {
            commands.commands.forEach(cmd => {
                const robotId = cmd.robot_id;
                
                // Extract skill information from state_factors
                if (cmd.state_factors && cmd.state_factors.length > 0) {
                    const skillFactor = cmd.state_factors.find(factor =>
                        factor.name && factor.state === 'RUNNING'
                    ) || cmd.state_factors[0];

                    if (skillFactor && skillFactor.name) {
                        this.executingSkills.set(robotId, {
                            skillName: skillFactor.name,
                            plannerName: cmd.planner_name || 'Unknown',
                            startTime: new Date(),
                            kickPower: cmd.kick_power || 0,
                            dribblePower: cmd.dribble_power || 0,
                            targetTheta: cmd.target_theta || 0,
                            chipEnable: cmd.chip_enable || false
                        });
                    }
                } else {
                    this.executingSkills.delete(robotId);
                }
            });
            
            this.scheduleGridUpdate();
            
            if (this.sidePanelOpen) {
                this.schedulePanelUpdate();
            }
        }
    }

    handleRobotDiagnostics(diagnostics) {
        if (diagnostics.diagnostics) {
            diagnostics.diagnostics.forEach(diag => {
                // Extract robot ID from hardware_id (format: "robot_X")
                const match = diag.hardware_id.match(/robot_(\d+)/);
                if (match) {
                    const robotId = parseInt(match[1]);
                    
                    if (!this.diagnostics.has(robotId)) {
                        this.diagnostics.set(robotId, {});
                    }
                    
                    this.diagnostics.get(robotId)[diag.name] = {
                        level: diag.level,
                        message: diag.message,
                        keyValues: diag.key_values || [],
                        timestamp: diagnostics.timestamp
                    };
                }
            });
            
            this.scheduleGridUpdate();
            
            if (this.sidePanelOpen) {
                this.schedulePanelUpdate();
            }
        }
    }

    handleRobotFeedback(feedback) {
        if (feedback.feedback) {
            feedback.feedback.forEach(robotFeedback => {
                this.robotFeedbacks.set(robotFeedback.robot_id, {
                    ...robotFeedback,
                    timestamp: feedback.timestamp
                });
            });
            
            this.scheduleGridUpdate();
            
            if (this.sidePanelOpen) {
                this.schedulePanelUpdate();
            }
        }
    }

    updateRobotData(robotId, robotData, team) {
        if (!this.robots.has(robotId)) {
            this.robots.set(robotId, {});
        }
        
        const robot = this.robots.get(robotId);
        
        // Update basic robot info
        robot.id = robotId;
        robot.team = team;
        robot.position = { x: robotData.x, y: robotData.y };
        robot.velocity = { x: robotData.vx, y: robotData.vy };
        robot.angle = robotData.theta;
        robot.speed = Math.sqrt(robotData.vx * robotData.vx + robotData.vy * robotData.vy);
        robot.detected = robotData.detected || false;
        robot.visionDetected = robotData.vision_detected || false;
        robot.feedbackDetected = robotData.feedback_detected || false;
        robot.lastUpdate = new Date();
        
        // Calculate ball distance
        if (this.worldModel && this.worldModel.ball) {
            const ballX = this.worldModel.ball.x;
            const ballY = this.worldModel.ball.y;
            robot.ballDistance = Math.sqrt(
                Math.pow(robotData.x - ballX, 2) + 
                Math.pow(robotData.y - ballY, 2)
            );
        }
    }

    scheduleGridUpdate() {
        const now = Date.now();
        if (now - this.lastGridUpdate >= this.gridUpdateInterval) {
            this.updateRobotGrid();
            this.lastGridUpdate = now;
        }
    }

    schedulePanelUpdate() {
        const now = Date.now();
        if (now - this.lastPanelUpdate >= this.panelUpdateInterval) {
            this.updateSidePanelContent();
            this.lastPanelUpdate = now;
        }
    }

    updateRobotGrid() {
        const grid = document.getElementById('robotGrid');
        const robotCountEl = document.getElementById('robotCount');
        
        // Filter robots based on team selection
        let filteredRobots = Array.from(this.robots.values());
        if (this.teamFilter === 'ours') {
            filteredRobots = filteredRobots.filter(robot => robot.team === 'ours');
        } else if (this.teamFilter === 'theirs') {
            filteredRobots = filteredRobots.filter(robot => robot.team === 'theirs');
        }
        
        robotCountEl.textContent = `(${filteredRobots.length} robots detected)`;
        
        // Clear existing cards
        grid.innerHTML = '';
        
        // Sort robots by ID
        filteredRobots.sort((a, b) => a.id - b.id);
        
        filteredRobots.forEach(robot => {
            const card = this.createRobotCard(robot);
            grid.appendChild(card);
        });
    }

    createRobotCard(robot) {
        const card = document.createElement('div');
        const robotId = robot.id;
        
        // Determine connection status
        let connectionClass = 'disconnected';
        let connectionText = 'Disconnected';
        
        if (robot.detected) {
            connectionClass = 'connected';
            connectionText = 'Connected';
        } else if (this.hasWarnings(robotId)) {
            connectionClass = 'warning';
            connectionText = 'Warning';
        }
        
        // Get team color
        const teamColorClass = this.getTeamColor(robot.team);
        
        // Get executing skill
        const skillInfo = this.executingSkills.get(robotId);
        const skillText = skillInfo ? `▶ ${skillInfo.skillName}` : 'Idle';
        const skillClass = skillInfo ? 'text-success' : 'text-muted';
        
        // Get battery level (from feedback if available)
        const feedback = this.robotFeedbacks.get(robotId);
        let batteryLevel = 0;
        let batteryClass = 'battery-unknown';
        
        if (feedback && feedback.voltage && feedback.voltage.length > 0) {
            // Assume first voltage is battery voltage, normalize to 0-100%
            const voltage = feedback.voltage[0];
            batteryLevel = Math.min(100, Math.max(0, (voltage - 11.0) / (12.6 - 11.0) * 100));
            
            if (batteryLevel > 60) batteryClass = 'battery-high';
            else if (batteryLevel > 30) batteryClass = 'battery-medium';
            else batteryClass = 'battery-low';
        }
        
        // Count errors and warnings
        const { errorCount, warningCount } = this.getErrorCounts(robotId);
        
        card.className = `robot-card ${connectionClass}`;
        card.onclick = () => this.openSidePanel(robotId);
        
        card.innerHTML = `
            <div class="robot-header">
                <div class="robot-id">
                    R${robotId}
                    <span class="team-indicator ${teamColorClass}"></span>
                </div>
            </div>
            
            <div class="connection-status ${connectionClass}">
                <span class="status-indicator status-${connectionClass === 'connected' ? 'ok' : connectionClass === 'warning' ? 'warning' : 'error'}"></span>
                ${connectionText}
            </div>
            
            <div class="skill-status ${skillClass}">
                ${skillText}
            </div>
            
            <div class="battery-container">
                <div class="battery-label">Battery: ${batteryLevel.toFixed(0)}%</div>
                <div class="battery-bar">
                    <div class="battery-fill ${batteryClass}" style="width: ${batteryLevel}%"></div>
                </div>
            </div>
            
            <div class="error-indicators">
                ${errorCount > 0 ? `<span class="error-badge">${errorCount}</span>` : ''}
                ${warningCount > 0 ? `<span class="error-badge warning-badge">${warningCount}</span>` : ''}
            </div>
        `;
        
        return card;
    }

    getTeamColor(team) {
        if (!this.worldModel) return 'team-blue';
        
        const isYellow = this.worldModel.is_yellow || false;
        
        if (team === 'ours') {
            return isYellow ? 'team-yellow' : 'team-blue';
        } else {
            return isYellow ? 'team-blue' : 'team-yellow';
        }
    }

    hasWarnings(robotId) {
        const diag = this.diagnostics.get(robotId);
        if (!diag) return false;
        
        return Object.values(diag).some(d => d.level === 1); // WARNING level
    }

    getErrorCounts(robotId) {
        const diag = this.diagnostics.get(robotId);
        let errorCount = 0;
        let warningCount = 0;
        
        if (diag) {
            Object.values(diag).forEach(d => {
                if (d.level === 2) errorCount++; // ERROR level
                else if (d.level === 1) warningCount++; // WARNING level
            });
        }
        
        return { errorCount, warningCount };
    }

    openSidePanel(robotId) {
        this.selectedRobotId = robotId;
        this.sidePanelOpen = true;
        
        document.getElementById('overlay').classList.add('show');
        document.getElementById('sidePanel').classList.add('open');
        
        this.updateSidePanelHeader(robotId);
        this.updateSidePanelContent();
    }

    closeSidePanel() {
        this.sidePanelOpen = false;
        this.selectedRobotId = null;
        
        document.getElementById('overlay').classList.remove('show');
        document.getElementById('sidePanel').classList.remove('open');
    }

    updateSidePanelHeader(robotId) {
        const robot = this.robots.get(robotId);
        if (!robot) return;
        
        const teamName = robot.team === 'ours' ? 'Our Team' : 'Their Team';
        const teamColor = this.getTeamColor(robot.team);
        
        document.getElementById('panelTitle').textContent = `Robot ${robotId}`;
        document.getElementById('panelSubtitle').innerHTML = `
            <span class="team-indicator ${teamColor}"></span>
            ${teamName}
        `;
    }

    updateSidePanelContent() {
        if (!this.selectedRobotId) return;
        
        const robot = this.robots.get(this.selectedRobotId);
        const feedback = this.robotFeedbacks.get(this.selectedRobotId);
        const diagnostics = this.diagnostics.get(this.selectedRobotId);
        const skillInfo = this.executingSkills.get(this.selectedRobotId);
        
        if (!robot) return;
        
        // Update Overview tab
        this.updateOverviewTab(robot);
        
        // Update Sensors tab
        this.updateSensorsTab(feedback);
        
        // Update Diagnostics tab
        this.updateDiagnosticsTab(diagnostics);
        
        // Update Commands tab
        this.updateCommandsTab(skillInfo);
    }

    updateOverviewTab(robot) {
        const elements = {
            'robotPosition': `(${robot.position.x.toFixed(3)}, ${robot.position.y.toFixed(3)})`,
            'robotVelocity': `(${robot.velocity.x.toFixed(3)}, ${robot.velocity.y.toFixed(3)})`,
            'robotAngle': `${robot.angle.toFixed(3)} rad (${(robot.angle * 180 / Math.PI).toFixed(1)}°)`,
            'robotSpeed': `${robot.speed.toFixed(3)} m/s`,
            'ballDistance': robot.ballDistance ? `${robot.ballDistance.toFixed(3)} m` : '-',
            'visionDetected': robot.visionDetected ? 
                '<span class="status-indicator status-ok"></span>Yes' : 
                '<span class="status-indicator status-error"></span>No',
            'feedbackDetected': robot.feedbackDetected ? 
                '<span class="status-indicator status-ok"></span>Yes' : 
                '<span class="status-indicator status-error"></span>No',
            'lastUpdate': robot.lastUpdate.toLocaleTimeString()
        };
        
        Object.entries(elements).forEach(([id, value]) => {
            const el = document.getElementById(id);
            if (el) el.innerHTML = value;
        });
    }

    updateSensorsTab(feedback) {
        if (!feedback) {
            // Clear sensor data
            const tempTable = document.getElementById('temperatureTable');
            if (tempTable) tempTable.innerHTML = '<tr><td colspan="2">No sensor data available</td></tr>';
            return;
        }
        
        // Update temperature table
        const tempTable = document.getElementById('temperatureTable');
        if (tempTable && feedback.temperatures) {
            const tempLabels = ['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'FET', 'Coil 1', 'Coil 2'];
            let tempHTML = '';
            
            feedback.temperatures.forEach((temp, index) => {
                const label = tempLabels[index] || `Sensor ${index}`;
                const tempClass = temp > 70 ? 'text-danger' : temp > 50 ? 'text-warning' : 'text-success';
                tempHTML += `<tr><td><strong>${label}:</strong></td><td class="${tempClass}">${temp}°C</td></tr>`;
            });
            
            tempTable.innerHTML = tempHTML || '<tr><td colspan="2">No temperature data</td></tr>';
        }
        
        // Update electrical status
        const elements = {
            'batteryVoltage': feedback.voltage && feedback.voltage.length > 0 ? 
                `${feedback.voltage[0].toFixed(2)}V` : '-',
            'motorCurrents': feedback.motor_current ? 
                feedback.motor_current.map(c => c.toFixed(2)).join(', ') + ' A' : '-',
            'ballSensorStatus': feedback.ball_sensor ? 
                '<span class="status-indicator status-ok"></span>Detected' : 
                '<span class="status-indicator status-error"></span>No Ball'
        };
        
        Object.entries(elements).forEach(([id, value]) => {
            const el = document.getElementById(id);
            if (el) el.innerHTML = value;
        });
    }

    updateDiagnosticsTab(diagnostics) {
        const container = document.getElementById('diagnosticsContent');
        if (!container) return;
        
        if (!diagnostics || Object.keys(diagnostics).length === 0) {
            container.innerHTML = '<p class="text-muted">No diagnostics data available</p>';
            return;
        }
        
        let html = '';
        Object.entries(diagnostics).forEach(([name, diag]) => {
            const levelClass = diag.level === 0 ? 'success' : diag.level === 1 ? 'warning' : 'danger';
            const levelText = diag.level === 0 ? 'OK' : diag.level === 1 ? 'WARNING' : 'ERROR';
            
            html += `
                <div class="card mb-3">
                    <div class="card-header">
                        <h6>
                            <span class="status-indicator status-${diag.level === 0 ? 'ok' : diag.level === 1 ? 'warning' : 'error'}"></span>
                            ${name.charAt(0).toUpperCase() + name.slice(1)}
                            <span class="badge bg-${levelClass} ms-2">${levelText}</span>
                        </h6>
                    </div>
                    <div class="card-body">
                        <p class="mb-2">${diag.message}</p>
                        ${diag.keyValues.length > 0 ? `
                            <table class="table table-sm">
                                ${diag.keyValues.map(kv => `<tr><td><strong>${kv.key}:</strong></td><td>${kv.value}</td></tr>`).join('')}
                            </table>
                        ` : ''}
                    </div>
                </div>
            `;
        });
        
        container.innerHTML = html;
    }

    updateCommandsTab(skillInfo) {
        const elements = {
            'currentSkill': skillInfo ? skillInfo.skillName : 'None',
            'currentPlanner': skillInfo ? skillInfo.plannerName : '-',
            'kickPower': skillInfo ? `${(skillInfo.kickPower * 100).toFixed(1)}%` : '-',
            'dribblePower': skillInfo ? `${(skillInfo.dribblePower * 100).toFixed(1)}%` : '-',
            'targetTheta': skillInfo ? `${skillInfo.targetTheta.toFixed(3)} rad` : '-'
        };
        
        Object.entries(elements).forEach(([id, value]) => {
            const el = document.getElementById(id);
            if (el) el.textContent = value;
        });
    }

    sendMessage(message) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(message));
        }
    }

    updateConnectionStatus(connected) {
        const statusIndicator = document.getElementById('connectionStatus');
        const statusText = document.getElementById('connectionText');

        if (connected) {
            statusIndicator.className = 'status-indicator status-ok';
            statusText.textContent = 'Connected';
        } else {
            statusIndicator.className = 'status-indicator status-error';
            statusText.textContent = 'Disconnected';
        }
    }

    addLog(message) {
        const timestamp = new Date().toLocaleTimeString();
        console.log(`[${timestamp}] ${message}`);
    }
}

// Initialize the application when the page loads
let robotStatusMonitor;
document.addEventListener('DOMContentLoaded', () => {
    robotStatusMonitor = new RobotStatusMonitor();
});