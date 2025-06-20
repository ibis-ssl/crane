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
        this.commandIndicatorTimeout = null;
        
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
        const wsUrl = 'ws://localhost:8091';
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
            this.addLog(`WebSocket connection failed: ${error.message}`, 'error');
            this.updateConnectionStatus(false);
            
            // Fallback to demo mode
            this.addLog('Falling back to demo mode', 'warning');
            this.availableSkills = [
                "Sleep", "Idle", "Kick", "Receive", "Goalie", "Attacker", "SubAttacker",
                "StealBall", "SingleBallPlacement", "GoalKick", "SimpleKickOff", 
                "KickOffAttack", "KickOffSupport", "Marker", "TestMotionPosition", 
                "TestMotionVelocity", "EmplaceRobot", "Forward", "BallNearbyPositioner",
                "GoOverBall", "SecondThreatDefender", "FreekickSaver", "PenaltyKick", "Teleop"
            ];
            this.populateSkillsList(this.availableSkills);
            this.addLog(`Demo mode: ${this.availableSkills.length} skills available`, 'info');
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
        // Update robot command visualization and show indicator
        this.showCommandIndicator();
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
            skillCard.addEventListener('click', (event) => {
                this.selectSkill(skill, event.target);
            });
            skillsList.appendChild(skillCard);
        });
    }

    selectSkill(skillName, targetElement = null) {
        this.selectedSkill = skillName;
        document.getElementById('selectedSkill').value = skillName;
        document.getElementById('executeBtn').disabled = false;
        
        // Remove active class from all skill cards
        document.querySelectorAll('.skill-card').forEach(card => {
            card.classList.remove('btn-primary');
            card.classList.add('btn-outline-primary');
        });
        
        // Add active class to selected skill if target element is provided
        if (targetElement) {
            targetElement.classList.remove('btn-outline-primary');
            targetElement.classList.add('btn-primary');
        }
        
        this.generateParametersForm(skillName);
        this.addLog(`Selected skill: ${skillName}`, 'info');
    }

    generateParametersForm(skillName) {
        const parametersForm = document.getElementById('parametersForm');
        
        // Define common parameters for different skills
        const skillParameters = {
            'Kick': [
                { name: 'target', type: 'text', label: 'Target (x,y)', value: '0,0' },
                { name: 'kick_power', type: 'number', label: 'Kick Power', min: '0', max: '1', step: '0.05', value: '0.7' },
                { name: 'use_target_kick_speed', type: 'checkbox', label: 'Use Target Kick Speed', value: 'false' },
                { name: 'target_kick_speed', type: 'number', label: 'Target Kick Speed', min: '0', max: '10', step: '0.1', value: '2.0' },
                { name: 'use_target_chip_distance', type: 'checkbox', label: 'Use Target Chip Distance', value: 'false' },
                { name: 'target_chip_distance', type: 'number', label: 'Target Chip Distance', min: '0', max: '10', step: '0.1', value: '2.0' },
                { name: 'chip_kick', type: 'checkbox', label: 'Chip Kick', value: 'false' },
                { name: 'with_dribble', type: 'checkbox', label: 'With Dribble', value: 'false' },
                { name: 'dribble_power', type: 'number', label: 'Dribble Power', min: '0', max: '1', step: '0.05', value: '0.3' },
                { name: 'angle_threshold', type: 'number', label: 'Angle Threshold', min: '0', max: '1', step: '0.01', value: '0.1' },
                { name: 'around_interval', type: 'number', label: 'Around Interval', min: '0', max: '1', step: '0.01', value: '0.15' },
                { name: 'go_around_ball', type: 'checkbox', label: 'Go Around Ball', value: 'true' },
                { name: 'moving_speed_threshold', type: 'number', label: 'Moving Speed Threshold', min: '0', max: '5', step: '0.1', value: '0.2' },
                { name: 'kicked_speed_threshold', type: 'number', label: 'Kicked Speed Threshold', min: '0', max: '10', step: '0.1', value: '1.5' }
            ],
            'EmplaceRobot': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' },
                { name: 'target_theta', type: 'number', label: 'Target Theta', step: '0.1' }
            ],
            'Sleep': [
                { name: 'duration', type: 'number', label: 'Duration (seconds)', min: '0', step: '0.1', value: '1.0' }
            ],
            'StealBall': [
                { name: 'steal_method', type: 'text', label: 'Steal Method', value: 'side' },
                { name: 'kicker_power', type: 'number', label: 'Kicker Power', min: '0', max: '1', step: '0.1', value: '0.4' }
            ],
            'SubAttacker': [
                { name: 'ball_vel_threshold', type: 'number', label: 'Ball Velocity Threshold', min: '0', max: '5', step: '0.1', value: '0.2' },
                { name: 'kicker_power', type: 'number', label: 'Kicker Power', min: '0', max: '1', step: '0.1', value: '0.8' }
            ],
            'SecondThreatDefender': [
                { name: 'offset', type: 'number', label: 'Offset', min: '0', max: '2', step: '0.1', value: '0.3' }
            ],
            'KickoffAttack': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1', value: '0.0' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1', value: '1.0' },
                { name: 'kick_power', type: 'number', label: 'Kick Power', min: '0', max: '1', step: '0.05', value: '0.25' }
            ],
            'BallNearbyPositioner': [
                { name: 'margin_distance', type: 'number', label: 'Margin Distance', min: '0', max: '5', step: '0.1' },
                { name: 'total_robot_number', type: 'number', label: 'Total Robot Number', min: '1', max: '15', step: '1' },
                { name: 'current_robot_index', type: 'number', label: 'Current Robot Index', min: '0', max: '14', step: '1' },
                { name: 'robot_interval', type: 'number', label: 'Robot Interval', min: '0.1', max: '2', step: '0.1' },
                { name: 'alternative_target_mode', type: 'checkbox', label: 'Alternative Target Mode', value: 'false' },
                { name: 'positioning_policy', type: 'text', label: 'Positioning Policy', value: 'auto' },
                { name: 'line_policy', type: 'text', label: 'Line Policy', value: 'arc' }
            ],
            'Forward': [
                { name: 'front_point', type: 'text', label: 'Front Point (x,y)', value: '0,0' },
                { name: 'back_point', type: 'text', label: 'Back Point (x,y)', value: '0,0' },
                { name: 'max_ball_distance', type: 'number', label: 'Max Ball Distance', min: '0', max: '10', step: '0.1' },
                { name: 'max_vel', type: 'number', label: 'Max Velocity', min: '0', max: '10', step: '0.1' }
            ],
            'GoOverBall': [
                { name: 'next_target_x', type: 'number', label: 'Next Target X', step: '0.1' },
                { name: 'next_target_y', type: 'number', label: 'Next Target Y', step: '0.1' },
                { name: 'margin', type: 'number', label: 'Margin', min: '0', max: '1', step: '0.05' },
                { name: 'reach_threshold', type: 'number', label: 'Reach Threshold', min: '0', max: '1', step: '0.05' }
            ],
            'Teleop': [
                { name: 'rotation_deg', type: 'number', label: 'Rotation (degrees)', min: '-180', max: '180', step: '1', value: '0' },
                { name: 'use_local_coordinate', type: 'checkbox', label: 'Use Local Coordinate', value: 'false' }
            ],
            'TestMotionPosition': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' }
            ],
            'TestMotionVelocity': [
                { name: 'velocity_x', type: 'number', label: 'Velocity X', step: '0.1' },
                { name: 'velocity_y', type: 'number', label: 'Velocity Y', step: '0.1' }
            ],
            'Receive': [
                { name: 'target_x', type: 'number', label: 'Target X', step: '0.1' },
                { name: 'target_y', type: 'number', label: 'Target Y', step: '0.1' }
            ],
            'Attacker': [
                { name: 'moving_ball_velocity', type: 'number', label: 'Moving Ball Velocity', min: '0', max: '5', step: '0.1', value: '1.0' },
                { name: 'robot_acc_for_prediction', type: 'number', label: 'Robot Acc for Prediction', min: '0', max: '10', step: '0.1', value: '2.5' },
                { name: 'robot_max_vel_for_prediction', type: 'number', label: 'Robot Max Vel for Prediction', min: '0', max: '10', step: '0.1', value: '5.0' }
            ],
            'Goalie': [
                { name: 'run_inplay', type: 'checkbox', label: 'Run in play', value: 'true' },
                { name: 'block_distance', type: 'number', label: 'Block Distance', min: '0.1', max: '2.0', step: '0.1', value: '0.5' },
                { name: 'robot_acc_for_prediction', type: 'number', label: 'Robot Acc for Prediction', min: '0.1', max: '10.0', step: '0.1', value: '2.0' },
                { name: 'robot_max_vel_for_prediction', type: 'number', label: 'Robot Max Vel for Prediction', min: '0.1', max: '10.0', step: '0.1', value: '5.0' }
            ],
            'SingleBallPlacement': [
                { name: 'placement_x', type: 'number', label: 'Placement X', step: '0.1', value: '0.0' },
                { name: 'placement_y', type: 'number', label: 'Placement Y', step: '0.1', value: '0.0' },
                { name: 'pass_enable', type: 'checkbox', label: 'Pass Enable', value: 'false' },
                { name: 'コート端判定のオフセット', type: 'number', label: 'Court Edge Offset', min: '0', max: '1', step: '0.05', value: '0.0' }
            ],
            'Marker': [
                { name: 'marking_robot_id', type: 'number', label: 'Marking Robot ID', min: '0', max: '15', step: '1', value: '0' },
                { name: 'mark_distance', type: 'number', label: 'Mark Distance', min: '0.1', max: '3.0', step: '0.1', value: '0.5' },
                { name: 'mark_mode', type: 'text', label: 'Mark Mode', value: 'save_goal' }
            ],
            'PenaltyKick': [
                { name: 'start_from_kick', type: 'checkbox', label: 'Start From Kick', value: 'false' },
                { name: 'prepare_margin', type: 'number', label: 'Prepare Margin', min: '0.1', max: '2.0', step: '0.1', value: '0.6' }
            ],
            'GoalKick': [
                { name: 'キック角度の最低要求精度[deg]', type: 'number', label: 'Kick Angle Accuracy (deg)', min: '0.1', max: '10', step: '0.1', value: '1.0' },
                { name: 'dribble_power', type: 'number', label: 'Dribble Power', min: '0', max: '1', step: '0.1', value: '0.0' }
            ]
        };
        
        const parameters = skillParameters[skillName] || [];
        
        if (parameters.length === 0) {
            parametersForm.innerHTML = '<p class="text-muted">No configurable parameters for this skill</p>';
            return;
        }
        
        let formHTML = '';
        parameters.forEach(param => {
            if (param.type === 'checkbox') {
                formHTML += `
                    <div class="mb-3 form-check">
                        <input type="checkbox" 
                               class="form-check-input" 
                               id="param_${param.name}" 
                               name="${param.name}"
                               ${param.value === 'true' ? 'checked' : ''}>
                        <label class="form-check-label" for="param_${param.name}">
                            ${param.label}
                        </label>
                    </div>
                `;
            } else {
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
            }
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
        
        // First, ensure simple_ai session is active
        this.sendMessage({
            type: 'activate_simple_ai'
        });
        
        // Small delay to allow session activation
        setTimeout(() => {
            const message = {
                type: 'execute_skill',
                skill_name: this.selectedSkill,
                robot_id: robotId,
                parameters: parameters
            };
            
            this.sendMessage(message);
            this.addLog(`Executing ${this.selectedSkill} on robot ${robotId}...`, 'info');
        }, 100);
    }

    collectParameters() {
        const parametersForm = document.getElementById('parametersForm');
        const inputs = parametersForm.querySelectorAll('input');
        const parameters = [];
        
        inputs.forEach(input => {
            if (input.type === 'checkbox') {
                // Always include checkbox parameters
                parameters.push({
                    name: input.name,
                    value: input.checked ? 'true' : 'false'
                });
            } else if (input.value.trim() !== '') {
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
        
        // Determine team colors based on is_yellow flag
        const isYellow = this.worldModel.is_yellow || false;
        const ourColor = isYellow ? '#ffc107' : '#007bff';  // Yellow or Blue
        const theirColor = isYellow ? '#007bff' : '#ffc107';  // Blue or Yellow
        
        // Draw ball
        if (this.worldModel.ball) {
            const ball = document.createElement('div');
            ball.className = 'ball';
            ball.style.left = `${(this.worldModel.ball.x + fieldWidth/2) * scaleX}px`;
            ball.style.top = `${(fieldHeight/2 - this.worldModel.ball.y) * scaleY}px`;
            field.appendChild(ball);
        }
        
        // Draw our robots
        if (this.worldModel.robots_ours) {
            this.worldModel.robots_ours.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-ours';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = ourColor;
                robotEl.title = `Our Robot ${robot.id}`;
                
                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';
                
                field.appendChild(robotEl);
            });
        }
        
        // Draw their robots
        if (this.worldModel.robots_theirs) {
            this.worldModel.robots_theirs.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-theirs';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = theirColor;
                robotEl.title = `Their Robot ${robot.id}`;
                
                // Add robot ID text
                robotEl.textContent = robot.id;
                robotEl.style.color = 'white';
                robotEl.style.fontSize = '10px';
                robotEl.style.textAlign = 'center';
                robotEl.style.lineHeight = '16px';
                
                field.appendChild(robotEl);
            });
        }
        
        // Fallback for old format (backward compatibility)
        if (this.worldModel.robots && !this.worldModel.robots_ours) {
            this.worldModel.robots.forEach(robot => {
                const robotEl = document.createElement('div');
                robotEl.className = 'robot robot-ours';
                robotEl.style.left = `${(robot.x + fieldWidth/2) * scaleX}px`;
                robotEl.style.top = `${(fieldHeight/2 - robot.y) * scaleY}px`;
                robotEl.style.backgroundColor = ourColor;
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
        if (!this.worldModel) {
            return;
        }
        
        const statusContainer = document.getElementById('robotStatus');
        let statusHTML = '';
        
        // Display our robots
        if (this.worldModel.robots_ours && this.worldModel.robots_ours.length > 0) {
            const isYellow = this.worldModel.is_yellow || false;
            const ourColorName = isYellow ? 'Yellow' : 'Blue';
            
            statusHTML += `<h6 class="text-primary">Our Robots (${ourColorName})</h6>`;
            this.worldModel.robots_ours.forEach(robot => {
                statusHTML += `
                    <div class="mb-2 p-2 border rounded" style="border-left: 4px solid ${isYellow ? '#ffc107' : '#007bff'};">
                        <strong>Robot ${robot.id}</strong><br>
                        <small>
                            Pos: (${robot.x.toFixed(2)}, ${robot.y.toFixed(2)})<br>
                            Vel: (${robot.vx.toFixed(2)}, ${robot.vy.toFixed(2)})<br>
                            θ: ${robot.theta.toFixed(2)} rad
                        </small>
                    </div>
                `;
            });
        }
        
        // Display their robots
        if (this.worldModel.robots_theirs && this.worldModel.robots_theirs.length > 0) {
            const isYellow = this.worldModel.is_yellow || false;
            const theirColorName = isYellow ? 'Blue' : 'Yellow';
            
            statusHTML += `<h6 class="text-danger mt-3">Their Robots (${theirColorName})</h6>`;
            this.worldModel.robots_theirs.forEach(robot => {
                statusHTML += `
                    <div class="mb-2 p-2 border rounded" style="border-left: 4px solid ${isYellow ? '#007bff' : '#ffc107'};">
                        <strong>Robot ${robot.id}</strong><br>
                        <small>
                            Pos: (${robot.x.toFixed(2)}, ${robot.y.toFixed(2)})<br>
                            Vel: (${robot.vx.toFixed(2)}, ${robot.vy.toFixed(2)})<br>
                            θ: ${robot.theta.toFixed(2)} rad
                        </small>
                    </div>
                `;
            });
        }
        
        // Fallback for old format
        if (statusHTML === '' && this.worldModel.robots && this.worldModel.robots.length > 0) {
            statusHTML += `<h6 class="text-primary">Robots</h6>`;
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
        }
        
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
        
        // Limit log entries to 500 for better performance
        const entries = logContainer.children;
        if (entries.length > 500) {
            logContainer.removeChild(entries[0]);
        }
    }

    clearLog() {
        document.getElementById('logContainer').innerHTML = '';
        this.addLog('Log cleared', 'info');
    }

    showCommandIndicator() {
        const indicator = document.getElementById('commandIndicator');
        const text = document.getElementById('commandText');
        
        // Show indicator as active
        indicator.className = 'command-indicator active';
        text.textContent = 'Command Received';
        
        // Clear previous timeout
        if (this.commandIndicatorTimeout) {
            clearTimeout(this.commandIndicatorTimeout);
        }
        
        // Auto-hide after 200ms
        this.commandIndicatorTimeout = setTimeout(() => {
            indicator.className = 'command-indicator';
            text.textContent = 'Waiting...';
        }, 200);
    }
}

// Initialize the application when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new CraneDebugger();
});